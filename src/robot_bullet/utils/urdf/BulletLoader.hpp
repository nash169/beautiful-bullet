#ifndef ROBOT_BULLET_UTILS_URDF_BULLETLOADER
#define ROBOT_BULLET_UTILS_URDF_BULLETLOADER

#include <string>

#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Featherstone/btMultiBody.h>

#include <urdf_parser/urdf_parser.h>
#include <urdf_world/world.h>

#include "robot_bullet/common/Console.hpp"
#include "robot_bullet/utils/urdf/MeshShape.hpp"

namespace robot_bullet {
    namespace utils {
        namespace urdf_parsing {
            class BulletLoader {
            public:
                BulletLoader() : _collisionMargin(0.001) {}

                ~BulletLoader() {}

                // URDF link: http://wiki.ros.org/urdf/XML/link
                // URDF joint: http://wiki.ros.org/urdf/XML/joint
                btMultiBody* parseMultiBody(const std::string& urdfString)
                {
                    if (urdfString.empty()) {
                        dtwarn << "[BulletLoader::parseMultiBody] A blank string cannot be "
                               << "parsed into a Skeleton. Returning a nullptr\n";
                        return nullptr;
                    }

                    urdf::ModelInterfaceSharedPtr urdfInterface = urdf::parseURDFFile(urdfString);

                    if (!urdfInterface) {
                        dtwarn << "[BulletLoader::parseMultiBody] Failed loading URDF.\n";
                        return nullptr;
                    }

                    // Store path
                    _path = urdfString;

                    return modelIntefaceToMultiBody(urdfInterface.get());
                }

            protected:
                btMultiBody* modelIntefaceToMultiBody(const urdf::ModelInterface* model)
                {
                    // std::cout << model->getName() << std::endl;

                    // Get root link (how to handle the case of multi-tree robot?)
                    const urdf::Link* root = model->getRoot().get();

                    // Set default mass, local inertia diagonal and number of links (base excluded)
                    int numLinks = model->links_.size() - 1;
                    btScalar mass;
                    btVector3 inertiaDiag;

                    if (root->name == "world") {
                        // Subtract world link to the number robot's link
                        numLinks -= 1;

                        // Move one link forward and ignore the world link
                        root = root->child_links[0].get();

                        // Set the mass of the base equal to zero
                        mass = 0;

                        // Set inertia to zero
                        inertiaDiag.setValue(0, 0, 0);
                    }
                    else {
                        // Get mass
                        btScalar mass = root->inertial->mass;

                        // Get inertia and diagonalize it
                        btVector3 inertiaDiag = inertiaDiagonal(root);
                    }

                    // Allocate MultiBody
                    btMultiBody* multibody = new btMultiBody(numLinks, mass, inertiaDiag, mass == 0, false);

                    // Base index
                    int index = 0;

                    // Recursively create the nodes
                    if (!createMultiBodyRecursive(model, multibody, root, index)) {
                        return nullptr;
                    }

                    return multibody;

                    // // Find mimic joints
                    // for (std::size_t i = 0; i < root->child_links.size(); i++)
                    //     addMimicJointsRecursive(model, skeleton, root->child_links[i].get());
                }

                bool createMultiBodyRecursive(const urdf::ModelInterface* model, btMultiBody* multibody, const urdf::Link* node, int index, int parentIndex = -1)
                {
                    std::cout << "Setting link: " << index << std::endl;

                    // If not base link create connection
                    if (parentIndex >= 0) {
                        if (!createJointNodeConnection(multibody, node, index, parentIndex))
                            return false;
                    }

                    // Create collision shape
                    if (!createNodeShapes(multibody, node))
                        return false;

                    // Recursively create the nodes
                    for (std::size_t i = 0; i < node->child_links.size(); ++i) {
                        if (!createMultiBodyRecursive(model, multibody, node->child_links[0].get(), index + 1 + i, index))
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
                    btTransform frameInertial = inertialFrame(node),
                                parentFrameInertial = inertialFrame(node->getParent().get());

                    // Get diagonalized inertia matrix
                    btVector3 inertiaDiag = inertiaDiagonal(node);

                    // Get joint pose
                    urdf::Pose pose = joint->parent_to_joint_origin_transform;

                    // Bullet transformation from parent to child
                    btTransform parentToJoint;
                    parentToJoint.setOrigin(btVector3(pose.position.x, pose.position.y, pose.position.z));
                    parentToJoint.setRotation(btQuaternion(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w));

                    // Get offsets
                    btTransform offsetInA = parentFrameInertial.inverse() * parentToJoint,
                                offsetInB = frameInertial.inverse();

                    // Get parent to joint rotation (check this)
                    btQuaternion parentRotToThis = offsetInB.getRotation() * offsetInA.inverse().getRotation();

                    // Set joint properties
                    multibody->getLink(index).m_jointDamping = joint->dynamics->damping;
                    multibody->getLink(index).m_jointFriction = joint->dynamics->friction;
                    multibody->getLink(index).m_jointLowerLimit = joint->limits->lower;
                    multibody->getLink(index).m_jointUpperLimit = joint->limits->upper;
                    multibody->getLink(index).m_jointMaxForce = joint->limits->effort;
                    multibody->getLink(index).m_jointMaxVelocity = joint->limits->velocity;

                    // Set parent-child collision
                    bool disableParentCollision = true;

                    // Create joint
                    switch (joint->type) {
                    case urdf::Joint::PLANAR:
                        /* code */
                        break;

                    case urdf::Joint::FLOATING:
                        /* code */
                        break;

                    case urdf::Joint::REVOLUTE: {
                        btVector3 jointAxis = btVector3(joint->axis.x, joint->axis.y, joint->axis.z);
                        multibody->setupRevolute(index, mass, inertiaDiag, parentIndex, parentRotToThis, quatRotate(offsetInB.getRotation(), jointAxis), offsetInA.getOrigin(), -offsetInB.getOrigin(), disableParentCollision);
                        break;
                    }
                    case urdf::Joint::CONTINUOUS: {
                        multibody->setupSpherical(index, mass, inertiaDiag, parentIndex, parentRotToThis, offsetInA.getOrigin(), -offsetInB.getOrigin(), disableParentCollision);
                        break;
                    }
                    case urdf::Joint::PRISMATIC:
                        /* code */
                        break;

                    case urdf::Joint::FIXED: {
                        multibody->setupFixed(index, mass, inertiaDiag, parentIndex, parentRotToThis, offsetInA.getOrigin(), -offsetInB.getOrigin(), disableParentCollision);
                        break;
                    }
                    default:
                        return false;
                    }

                    return true;
                }

                bool createNodeShapes(btMultiBody* multibody, const urdf::Link* node)
                {
                    // Visual shape (this should not be active without graphics)
                    // for (auto& visual : node->visual_array) {
                    //     createShape(visual.get());
                    // }

                    // Allocate compound shape
                    btCompoundShape* compoundShape = new btCompoundShape();

                    // Transformation (to do...)
                    btTransform inertiaFrame;
                    inertiaFrame.setIdentity();

                    // Collision shape
                    for (auto& collision : node->collision_array) {
                        createShape(collision.get());
                        compoundShape->addChildShape(inertiaFrame, createShape(collision.get()));
                    }

                    return true;
                }

                template <typename ShapeType>
                btCollisionShape* createShape(const ShapeType* shape)
                {
                    // Allocate collision shape
                    btCollisionShape* collisionShape;

                    // Check shape type
                    switch (shape->geometry->type) {
                    case urdf::Geometry::SPHERE:
                        std::cout << "Sphere" << std::endl;
                        break;

                    case urdf::Geometry::BOX:
                        std::cout << "Box" << std::endl;
                        break;

                    case urdf::Geometry::CYLINDER:
                        std::cout << "Cylinder" << std::endl;
                        break;

                    case urdf::Geometry::MESH: {
                        urdf::Mesh* mesh = dynamic_cast<urdf::Mesh*>(shape->geometry.get());

                        // In this way the Assimp importer should be alive till the creation of the collision shape
                        MeshShape importer;

                        const aiScene* scene = importer.loadMesh("models/iiwa/" + mesh->filename);
                        btVector3 scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);

                        std::cout << mesh->filename << std::endl;

                        collisionShape = createConvexHullFromShapes(scene, scale);

                        break;
                    }

                    default:
                        return nullptr;
                    }

                    return collisionShape;
                }

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

                btTransform inertialFrame(const urdf::Link* node)
                {
                    btTransform frame;

                    frame.setOrigin(btVector3(node->inertial->origin.position.x, node->inertial->origin.position.y, node->inertial->origin.position.z));

                    frame.setRotation(btQuaternion(node->inertial->origin.rotation.x, node->inertial->origin.rotation.y, node->inertial->origin.rotation.z, node->inertial->origin.rotation.w));

                    return frame;
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

                        compound->addChildShape(identity, convexHull);
                    }

                    return compound;
                }

                // URDF path
                std::string _path;

                // Collision margin
                btScalar _collisionMargin;
            };
        } // namespace urdf_parsing
    } // namespace utils
} // namespace robot_bullet

#endif // ROBOT_BULLET_UTILS_URDF_BULLETLOADER