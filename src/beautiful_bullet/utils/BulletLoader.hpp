/*
    This file is part of beautiful-bullet.

    Copyright (c) 2021, 2022 Bernardo Fichera <bernardo.fichera@gmail.com>

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef BEAUTIFUL_BULLET_UTILS_URDF_BULLETLOADER
#define BEAUTIFUL_BULLET_UTILS_URDF_BULLETLOADER

// std library
#include <string>

// bullet
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Featherstone/btMultiBody.h>
// #include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>

// urdfdom
#include <urdf_parser/urdf_parser.h>

// beautiful bullet
#include "beautiful_bullet/tools/msg.hpp"
#include "beautiful_bullet/utils/LoadFlags.hpp"
#include "beautiful_bullet/utils/MeshShape.hpp"

namespace beautiful_bullet {
    namespace utils {
        class BulletLoader {
        public:
            BulletLoader() : _collisionMargin(0.001) {}

            ~BulletLoader() {}

            // URDF link: http://wiki.ros.org/urdf/XML/link
            // URDF joint: http://wiki.ros.org/urdf/XML/joint
            btMultiBody* parseMultiBody(const std::string& file, const int& flags, const bool& fixedBase = false) // Passing the world (change this)
            {
                // Check if the string is empty
                if (file.empty()) {
                    std::cerr << "Empty string." << std::endl;
                    return nullptr;
                }

                // Get path
                size_t pos = file.find_last_of("\\/");
                _path = (std::string::npos == pos) ? "" : file.substr(0, pos + 1);

                // Get model
                _model = urdf::parseURDFFile(file);
                if (!_model) {
                    std::cerr << "Failed loading URDF." << std::endl;
                    return nullptr;
                }

                // Store flags
                _flags = flags;

                return modelIntefaceToMultiBody(fixedBase);
            }

            // get model
            urdf::ModelInterfaceSharedPtr model() const { return _model; }

            // get path
            const std::string& path() const { return _path; };

        protected:
            // URDF path
            std::string _path;

            // Model
            urdf::ModelInterfaceSharedPtr _model;

            // Collision margin
            btScalar _collisionMargin;

            // Flags
            int _flags;

            btMultiBody* modelIntefaceToMultiBody(const bool& fixedBase)
            {
                // Get root link (how to handle the case of multi-tree robot?)
                const urdf::Link* root = _model->getRoot().get();

                // Set default mass, local inertia diagonal and number of links (base excluded)
                int numLinks = _model->links_.size() - 1;

                // Mass and diagonal inertia
                btScalar mass = 0;
                btVector3 inertiaDiag(0, 0, 0);

                if (root->name == "world") {
                    DEBUG_MSG("World link present: move forward one link");

                    // Subtract world link to the number robot's link
                    numLinks -= 1;

                    // Move one link forward and ignore the world link
                    root = root->child_links[0].get();
                }
                else {
                    if (!fixedBase) {
                        // Get mass
                        mass = root->inertial->mass;

                        // Get inertia and diagonalize it
                        inertiaDiag = inertiaDiagonal(root);
                    }
                }

                // Allocate MultiBody
                bool canSleep = (_flags & CUF_ENABLE_SLEEPING) != 0;
                btMultiBody* multibody = new btMultiBody(numLinks, mass, inertiaDiag, mass == 0, canSleep);

                if (_flags & CUF_GLOBAL_VELOCITIES_MB)
                    multibody->useGlobalVelocities(true);

                // if (_flags & CUF_USE_MJCF)
                // 	multibody->setBaseWorldTransform(linkTransformInWorldSpace);

                // Recursively create the nodes
                if (!createMultiBodyRecursive(multibody, root)) {
                    std::cerr << "error" << std::endl;
                    return nullptr;
                }

                // Finalize multibody
                multibody->setHasSelfCollision((_flags & CUF_USE_SELF_COLLISION) != 0);

                multibody->finalizeMultiDof();

                multibody->setBaseWorldTransform(inertiaFrame(root));

                btAlignedObjectArray<btQuaternion> scratch_q;
                btAlignedObjectArray<btVector3> scratch_m;
                multibody->forwardKinematics(scratch_q, scratch_m);
                multibody->updateCollisionObjectWorldTransforms(scratch_q, scratch_m);

                return multibody;
            }

            bool createMultiBodyRecursive(btMultiBody* multibody, const urdf::Link* node, int index = -1, int parentIndex = -2)
            {
                DEBUG_MSG(node->name);
                DEBUG_MSG("Setting link: " << index << " Parent link: " << parentIndex);

                // Create collision shape
                DEBUG_MSG("Creating node shape...");
                if (!createNodeShapes(multibody, node, index)) {
                    return false;
                }

                // If not base link create connection
                if (index >= 0) {
                    DEBUG_MSG("Creating connection...");
                    if (!createJointNodeConnection(multibody, node, index, parentIndex))
                        return false;
                }

                // Recursively create the nodes
                for (std::size_t i = 0; i < node->child_links.size(); ++i) {
                    if (!createMultiBodyRecursive(multibody, node->child_links[i].get(), index + 1 + i, index))
                        return false;
                }

                return true;
            }

            bool createJointNodeConnection(btMultiBody* multibody, const urdf::Link* node, int index, int parentIndex)
            {
                // Get parent joint
                const urdf::Joint* joint = node->parent_joint.get();

                // Get mass
                btScalar mass = node->inertial->mass;

                // Get inertial frame child and parent link
                btTransform frameInertial = inertiaFrame(node),
                            parentFrameInertial = inertiaFrame(node->getParent().get());

                // Get diagonalized inertia matrix
                btVector3 inertiaDiag(0, 0, 0);
                if (mass) {
                    if (_flags & CUF_USE_URDF_INERTIA) {
                        // Calculate diagonalized inertia
                        inertiaDiag = inertiaDiagonal(node);
                    }
                    else {
                        // Get diagonalized inertia from collision shapes
                        multibody->getLinkCollider(index)->getCollisionShape()->calculateLocalInertia(mass, inertiaDiag);

                        // Assert nan values
                        btAssert(inertiaDiag[0] < 1e10);
                        btAssert(inertiaDiag[1] < 1e10);
                        btAssert(inertiaDiag[2] < 1e10);
                    }
                }

                // Transformation from parent to child
                btTransform parentToJoint = jointFrame(joint);

                // Get offsets
                btTransform offsetInA = parentFrameInertial.inverse() * parentToJoint,
                            offsetInB = frameInertial.inverse();

                // Get parent to joint rotation (check this)
                btQuaternion parentRotToThis = offsetInB.getRotation() * offsetInA.inverse().getRotation();

                // Set joint properties
                if (joint->type != urdf::Joint::FIXED) {
                    multibody->getLink(index).m_jointDamping = joint->dynamics->damping;
                    multibody->getLink(index).m_jointFriction = joint->dynamics->friction;
                    multibody->getLink(index).m_jointLowerLimit = joint->limits->lower;
                    multibody->getLink(index).m_jointUpperLimit = joint->limits->upper;
                    multibody->getLink(index).m_jointMaxForce = joint->limits->effort;
                    multibody->getLink(index).m_jointMaxVelocity = joint->limits->velocity;
                }

                // Set parent-child collision
                bool disableParentCollision = true;

                // Joint axis
                btVector3 jointAxis = btVector3(joint->axis.x, joint->axis.y, joint->axis.z);

                // Create joint
                switch (joint->type) {
                case urdf::Joint::PLANAR:
                    DEBUG_MSG("Setting planar joint...");
                    multibody->setupPlanar(index, mass, inertiaDiag, parentIndex, parentRotToThis, quatRotate(offsetInB.getRotation(), jointAxis), offsetInA.getOrigin(), disableParentCollision);
                    break;

                case urdf::Joint::FLOATING:
                    std::cerr << "Floating not supported." << std::endl;
                    break;

                case urdf::Joint::REVOLUTE: {
                    DEBUG_MSG("Setting revolute joint...");
                    multibody->setupRevolute(index, mass, inertiaDiag, parentIndex,
                        parentRotToThis, quatRotate(offsetInB.getRotation(), jointAxis), offsetInA.getOrigin(), -offsetInB.getOrigin(), disableParentCollision);
                    break;
                }
                case urdf::Joint::CONTINUOUS: {
                    DEBUG_MSG("Setting continuous joint...");
                    multibody->setupSpherical(index, mass, inertiaDiag, parentIndex, parentRotToThis, offsetInA.getOrigin(), -offsetInB.getOrigin(), disableParentCollision);
                    break;
                }
                case urdf::Joint::PRISMATIC:
                    DEBUG_MSG("Setting prismatic joint...");
                    multibody->setupPrismatic(index, mass, inertiaDiag, parentIndex,
                        parentRotToThis, quatRotate(offsetInB.getRotation(), jointAxis), offsetInA.getOrigin(), -offsetInB.getOrigin(), disableParentCollision);
                    break;

                case urdf::Joint::FIXED: {
                    DEBUG_MSG("Setting fixed joint...");
                    multibody->setupFixed(index, mass, inertiaDiag, parentIndex, parentRotToThis, offsetInA.getOrigin(), -offsetInB.getOrigin(), disableParentCollision);
                    break;
                }
                default:
                    return false;
                }

                return true;
            }

            bool createNodeShapes(btMultiBody* multibody, const urdf::Link* node, const int& index)
            {
                // Transformation (to do...)
                btTransform parentToJoint, worldTranformation;
                worldTranformation.setIdentity();
                if (index >= 0) {
                    parentToJoint = jointFrame(node->parent_joint.get());
                    jointFrameWorld(node, index, worldTranformation);
                }
                else
                    parentToJoint.setIdentity();

                // Allocate compound shape
                btCompoundShape* compoundShape = new btCompoundShape();

                for (auto& collision : node->collision_array)
                    compoundShape->addChildShape(inertiaFrame(node).inverse() * collisionFrame(collision.get()), createShape(collision.get()));

                // std::cout << "LINK: " << index << " NUM: " << compoundShape->getNumChildShapes() << std::endl;
                // btTransform temp = compoundShape->getChildTransform(0);
                // std::cout << temp.getOrigin().x() << " " << temp.getOrigin().y() << " " << temp.getOrigin().z() << std::endl;
                // std::cout << temp.getRotation().x() << " " << temp.getRotation().y() << " " << temp.getRotation().z() << " " << temp.getRotation().w() << std::endl;

                btCollisionShape* collisionShape = compoundShape;
                if (compoundShape->getNumChildShapes() == 1 && compoundShape->getChildTransform(0) == btTransform::getIdentity())
                    collisionShape = compoundShape->getChildShape(0);

                // // Set local inertial collision shape
                // btVector3 inertiaDiag = inertiaDiagonal(node);
                // collisionShape->calculateLocalInertia(node->inertial->mass, inertiaDiag);

                // Create multibody link collider
                btMultiBodyLinkCollider* collider = new btMultiBodyLinkCollider(multibody, index);

                // Set collision shape
                collider->setCollisionShape(collisionShape);

                if (compoundShape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE) {
                    btBvhTriangleMeshShape* trimeshShape = (btBvhTriangleMeshShape*)compoundShape;
                    if (trimeshShape->getTriangleInfoMap())
                        collider->setCollisionFlags(collider->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
                }

                if (compoundShape->getShapeType() == TERRAIN_SHAPE_PROXYTYPE)
                    collider->setCollisionFlags(collider->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);

                // Set world transformation
                collider->setWorldTransform(worldTranformation);

                // Set contact friction (check here)
                collider->setFriction(0.5);

                // Deactivate collision
                // collider->setCollisionFlags(collider->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);

                // Set collider in the multibody
                if (index >= 0) {
                    if (!multibody->getBaseMass()) {
                        bool allJointsFixed = true;
                        int currIndex = index;
                        do {
                            if (multibody->getLink(currIndex).m_jointType != btMultibodyLink::eFixed) {
                                allJointsFixed = false;
                                break;
                            }
                            currIndex = multibody->getLink(currIndex).m_parent;
                        } while (currIndex > 0);

                        if (allJointsFixed)
                            collider->setCollisionFlags(collider->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
                    }

                    multibody->getLink(index).m_collider = collider;

                    if (_flags & CUF_USE_SELF_COLLISION_INCLUDE_PARENT) {
                        multibody->getLink(index).m_flags &= ~BT_MULTIBODYLINKFLAGS_DISABLE_PARENT_COLLISION;
                    }
                    if (_flags & CUF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS) {
                        multibody->getLink(index).m_flags |= BT_MULTIBODYLINKFLAGS_DISABLE_ALL_PARENT_COLLISION;
                    }
                }
                else {
                    if (!multibody->getBaseMass())
                        collider->setCollisionFlags(collider->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);

                    multibody->setBaseCollider(collider);
                }

                return true;
            }

            btCollisionShape* createShape(const urdf::Collision* shape)
            {
                // Allocate collision shape
                btCollisionShape* collisionShape;

                // Check shape type
                switch (shape->geometry->type) {
                case urdf::Geometry::SPHERE: {
                    urdf::Sphere* sphere = dynamic_cast<urdf::Sphere*>(shape->geometry.get());

                    btSphereShape* sphereShape = new btSphereShape(sphere->radius);
                    collisionShape = sphereShape;
                    collisionShape->setMargin(_collisionMargin);

                    break;
                }
                case urdf::Geometry::BOX: {
                    urdf::Box* box = dynamic_cast<urdf::Box*>(shape->geometry.get());

                    btBoxShape* boxShape = new btBoxShape(0.5 * btVector3(box->dim.x, box->dim.y, box->dim.z));

                    if (_flags & CUF_INITIALIZE_SAT_FEATURES)
                        boxShape->initializePolyhedralFeatures();

                    collisionShape = boxShape;
                    collisionShape->setMargin(_collisionMargin);
                    break;
                }
                case urdf::Geometry::CYLINDER: {
                    urdf::Cylinder* cylinder = dynamic_cast<urdf::Cylinder*>(shape->geometry.get());

                    if (_flags & CUF_USE_IMPLICIT_CYLINDER) {
                        btCylinderShapeZ* cylinderShape = new btCylinderShapeZ(btVector3(cylinder->radius, cylinder->radius, 0.5 * cylinder->length));
                        collisionShape = cylinderShape;
                        collisionShape->setMargin(_collisionMargin);
                    }
                    else {
                        btAlignedObjectArray<btVector3> vertices;
                        int numSteps = 32;
                        for (int i = 0; i < numSteps; i++) {
                            btVector3 vert(cylinder->radius * btSin(SIMD_2_PI * (float(i) / numSteps)), cylinder->radius * btCos(SIMD_2_PI * (float(i) / numSteps)), 0.5 * cylinder->length);
                            vertices.push_back(vert);
                            vert[2] = -(0.5 * cylinder->length);
                            vertices.push_back(vert);
                        }
                        btConvexHullShape* cylinderShape = new btConvexHullShape(&vertices[0].x(), vertices.size(), sizeof(btVector3));
                        cylinderShape->setMargin(_collisionMargin);
                        cylinderShape->recalcLocalAabb();

                        if (_flags & CUF_INITIALIZE_SAT_FEATURES)
                            cylinderShape->initializePolyhedralFeatures();

                        cylinderShape->optimizeConvexHull();
                        collisionShape = cylinderShape;
                    }
                    break;
                }
                case urdf::Geometry::MESH: {
                    urdf::Mesh* mesh = dynamic_cast<urdf::Mesh*>(shape->geometry.get());

                    // In this way the Assimp importer should be alive till the creation of the collision shape
                    MeshShape importer;

                    const aiScene* scene = importer.loadMesh(_path + mesh->filename);
                    btVector3 scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);

                    DEBUG_MSG(mesh->filename);

                    collisionShape = createConvexHullFromShapes(scene, scale);

                    break;
                }
                default: {
                    std::cerr << "Shape not found." << std::endl;
                    return nullptr;
                }
                }

                return collisionShape;
            }

            // Assimp obj structure: https://sites.google.com/site/gsucomputergraphics/educational/load_3d_model
            btCollisionShape* createConvexHullFromShapes(const aiScene* scene, const btVector3& scale)
            {
                btCompoundShape* compound = new btCompoundShape();
                compound->setMargin(_collisionMargin);

                btTransform identity;
                identity.setIdentity();

                // std::cout << scene->mNumMeshes << std::endl;
                // std::cout << scene->mMeshes[0]->mNumFaces << std::endl;
                // std::cout << scene->mMeshes[0]->mFaces[0].mNumIndices << std::endl;

                for (size_t i = 0; i < scene->mNumMeshes; i++) {
                    btConvexHullShape* convexHull = new btConvexHullShape();
                    convexHull->setMargin(_collisionMargin);

                    for (size_t j = 0; j < scene->mMeshes[i]->mNumFaces; j++) {
                        btVector3 pt;

                        for (size_t k = 0; k < scene->mMeshes[i]->mFaces[j].mNumIndices; k++) {
                            unsigned int index = scene->mMeshes[i]->mFaces[j].mIndices[k];

                            pt.setValue(
                                scene->mMeshes[i]->mVertices[index].x,
                                scene->mMeshes[i]->mVertices[index].y,
                                scene->mMeshes[i]->mVertices[index].z);

                            convexHull->addPoint(pt * scale, false);
                        }
                    }

                    convexHull->recalcLocalAabb();
                    convexHull->optimizeConvexHull();

                    // if (flags & CUF_INITIALIZE_SAT_FEATURES)
                    //     convexHull->initializePolyhedralFeatures();

                    compound->addChildShape(identity, convexHull);
                }

                return compound;
            }

            /* Parent joint -> joint */
            btTransform jointFrame(const urdf::Joint* joint)
            {
                // Get joint pose
                urdf::Pose pose = joint->parent_to_joint_origin_transform;

                // Bullet transformation from parent to child
                btTransform parentToJoint;
                parentToJoint.setIdentity();
                parentToJoint.setOrigin(btVector3(pose.position.x, pose.position.y, pose.position.z));
                parentToJoint.setRotation(btQuaternion(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w));

                return parentToJoint;
            }

            /* Base -> Node's parent joint */
            void jointFrameWorld(const urdf::Link* node, int index, btTransform& frameWorld)
            {
                // std::cout << "hello" << std::endl;
                frameWorld = jointFrame(node->parent_joint.get()) * frameWorld;
                index = index - node->getParent()->child_links.size();

                if (index >= 0) {
                    jointFrameWorld(node->getParent().get(), index, frameWorld);
                }
            }

            /* Node's parent joint -> inertia frame */
            btTransform inertiaFrame(const urdf::Link* node)
            {
                btTransform frame;
                frame.setIdentity();
                frame.setOrigin(btVector3(node->inertial->origin.position.x, node->inertial->origin.position.y, node->inertial->origin.position.z));
                frame.setRotation(btQuaternion(node->inertial->origin.rotation.x, node->inertial->origin.rotation.y, node->inertial->origin.rotation.z, node->inertial->origin.rotation.w));

                return frame;
            }

            /* Node's parent joint -> collsion frame */
            btTransform collisionFrame(const urdf::Collision* collision)
            {
                btTransform frame;
                frame.setIdentity();
                frame.setOrigin(btVector3(collision->origin.position.x, collision->origin.position.y, collision->origin.position.z));
                frame.setRotation(btQuaternion(collision->origin.rotation.x, collision->origin.rotation.y, collision->origin.rotation.z, collision->origin.rotation.w));

                return frame;
            }

            /* Node's parent joint -> visual frame */
            btTransform visualFrame(const urdf::Visual* visual)
            {
                btTransform frame;
                frame.setIdentity();
                frame.setOrigin(btVector3(visual->origin.position.x, visual->origin.position.y, visual->origin.position.z));
                frame.setRotation(btQuaternion(visual->origin.rotation.x, visual->origin.rotation.y, visual->origin.rotation.z, visual->origin.rotation.w));

                return frame;
            }

            /* Diagonalize inertial matrix */
            btVector3 inertiaDiagonal(const urdf::Link* node, const btScalar& threshold = 1e-6, const int& numIterations = 30)
            {
                btVector3 inertiaDiag;

                if (node->inertial->ixy == 0 && node->inertial->ixz == 0 && node->inertial->iyz == 0)
                    inertiaDiag.setValue(node->inertial->ixx, node->inertial->iyy, node->inertial->izz);
                else {
                    btMatrix3x3 linkInertiaBasis,
                        inertiaTensor(node->inertial->ixx, node->inertial->ixy, node->inertial->ixz,
                            node->inertial->ixy, node->inertial->iyy, node->inertial->iyz,
                            node->inertial->ixz, node->inertial->iyz, node->inertial->izz);

                    linkInertiaBasis.setIdentity();

                    inertiaTensor.diagonalize(linkInertiaBasis, threshold, numIterations);

                    inertiaDiag.setValue(inertiaTensor[0][0], inertiaTensor[1][1], inertiaTensor[2][2]);
                }

                return inertiaDiag;
            }
        };
    } // namespace utils
} // namespace beautiful_bullet

#endif // BEAUTIFUL_BULLET_UTILS_URDF_BULLETLOADER