#include "robot_bullet/importers/ImporterURDF.hpp"
#include "robot_bullet/importers/ImportMesh.hpp"

namespace robot_bullet {
    namespace importers {
        static btScalar gUrdfDefaultCollisionMargin = 0.001;

        std::vector<std::vector<std::string>> ImporterURDF::getLinkMeshes()
        {
            std::vector<std::vector<std::string>> links_meshes;

            for (size_t i = 0; i < m_data->m_urdfParser.getModel().m_links.size(); i++) {
                std::vector<std::string> meshes;

                UrdfLink* linkPtr = *m_data->m_urdfParser.getModel().m_links.getAtIndex(i);

                for (size_t j = 0; j < linkPtr->m_visualArray.size(); j++)
                    meshes.push_back(linkPtr->m_visualArray[j].m_geometry.m_meshFileName);

                // links_meshes.push_back(links_meshes);
            }

            return links_meshes;
        }

        bool ImporterURDF::loadURDF(const char* fileName, bool forceFixedBase)
        {
            if (strlen(fileName) == 0)
                return false;

            //int argc=0;
            char relativeFileName[1024];

            b3FileUtils fu;

            //bool fileFound = fu.findFile(fileName, relativeFileName, 1024);
            bool fileFound = m_data->m_fileIO->findResourcePath(fileName, relativeFileName, 1024);

            std::string xml_string;

            if (!fileFound) {
                b3Warning("URDF file '%s' not found\n", fileName);
                return false;
            }
            else {
                char path[1024];
                fu.extractPath(relativeFileName, path, sizeof(path));
                m_data->setSourceFile(relativeFileName, path);

                //read file
                int fileId = m_data->m_fileIO->fileOpen(relativeFileName, "r");

                char destBuffer[8192];
                char* line = 0;
                do {
                    line = m_data->m_fileIO->readLine(fileId, destBuffer, 8192);
                    if (line) {
                        xml_string += (std::string(destBuffer) + "\n");
                    }
                } while (line);
                m_data->m_fileIO->fileClose(fileId);
            }

            BulletErrorLogger loggie;
            m_data->m_urdfParser.setParseSDF(false);
            bool result = false;

            if (xml_string.length()) {
                result = m_data->m_urdfParser.loadUrdf(xml_string.c_str(), &loggie, forceFixedBase, (m_data->m_flags & CUF_PARSE_SENSORS));

                if (m_data->m_flags & CUF_IGNORE_VISUAL_SHAPES) {
                    for (int i = 0; i < m_data->m_urdfParser.getModel().m_links.size(); i++) {
                        UrdfLink* linkPtr = *m_data->m_urdfParser.getModel().m_links.getAtIndex(i);
                        linkPtr->m_visualArray.clear();
                    }
                }
                if (m_data->m_flags & CUF_IGNORE_COLLISION_SHAPES) {
                    for (int i = 0; i < m_data->m_urdfParser.getModel().m_links.size(); i++) {
                        UrdfLink* linkPtr = *m_data->m_urdfParser.getModel().m_links.getAtIndex(i);
                        linkPtr->m_collisionArray.clear();
                    }
                }
                if (m_data->m_urdfParser.getModel().m_rootLinks.size()) {
                    if (m_data->m_flags & CUF_MERGE_FIXED_LINKS) {
                        m_data->m_urdfParser.mergeFixedLinks(m_data->m_urdfParser.getModel(), m_data->m_urdfParser.getModel().m_rootLinks[0], &loggie, forceFixedBase, 0);
                        m_data->m_urdfParser.getModel().m_links.clear();
                        m_data->m_urdfParser.getModel().m_joints.clear();
                        m_data->m_urdfParser.recreateModel(m_data->m_urdfParser.getModel(), m_data->m_urdfParser.getModel().m_rootLinks[0], &loggie);
                    }
                    if (m_data->m_flags & CUF_PRINT_URDF_INFO) {
                        m_data->m_urdfParser.printTree(m_data->m_urdfParser.getModel().m_rootLinks[0], &loggie, 0);
                    }
                }
            }

            return result;
        }

        int ImporterURDF::getRootLinkIndex() const
        {
            if (m_data->m_urdfParser.getModel().m_rootLinks.size() == 1) {
                return m_data->m_urdfParser.getModel().m_rootLinks[0]->m_linkIndex;
            }
            return -1;
        };

        void ImporterURDF::getLinkChildIndices(int linkIndex, btAlignedObjectArray<int>& childLinkIndices) const
        {
            childLinkIndices.resize(0);
            UrdfLink* const* linkPtr = m_data->m_urdfParser.getModel().m_links.getAtIndex(linkIndex);
            if (linkPtr) {
                const UrdfLink* link = *linkPtr;
                //int numChildren = m_data->m_urdfParser->getModel().m_links.getAtIndex(linkIndex)->

                for (int i = 0; i < link->m_childLinks.size(); i++) {
                    int childIndex = link->m_childLinks[i]->m_linkIndex;
                    childLinkIndices.push_back(childIndex);
                }
            }
        }

        void ImporterURDF::getMassAndInertia(int urdfLinkIndex, btScalar& mass, btVector3& localInertiaDiagonal, btTransform& inertialFrame, int flags) const
        {
            if (flags & CUF_USE_URDF_INERTIA) {
                //the link->m_inertia is NOT necessarily aligned with the inertial frame
                //so an additional transform might need to be computed
                UrdfLink* const* linkPtr = m_data->m_urdfParser.getModel().m_links.getAtIndex(urdfLinkIndex);

                btAssert(linkPtr);
                if (linkPtr) {
                    UrdfLink* link = *linkPtr;
                    btMatrix3x3 linkInertiaBasis;
                    btScalar linkMass, principalInertiaX, principalInertiaY, principalInertiaZ;
                    if (link->m_parentJoint == 0 && m_data->m_urdfParser.getModel().m_overrideFixedBase) {
                        linkMass = 0.f;
                        principalInertiaX = 0.f;
                        principalInertiaY = 0.f;
                        principalInertiaZ = 0.f;
                        linkInertiaBasis.setIdentity();
                    }
                    else {
                        linkMass = link->m_inertia.m_mass;
                        if (link->m_inertia.m_ixy == 0.0 && link->m_inertia.m_ixz == 0.0 && link->m_inertia.m_iyz == 0.0) {
                            principalInertiaX = link->m_inertia.m_ixx;
                            principalInertiaY = link->m_inertia.m_iyy;
                            principalInertiaZ = link->m_inertia.m_izz;
                            linkInertiaBasis.setIdentity();
                        }
                        else {
                            principalInertiaX = link->m_inertia.m_ixx;
                            btMatrix3x3 inertiaTensor(link->m_inertia.m_ixx, link->m_inertia.m_ixy, link->m_inertia.m_ixz,
                                link->m_inertia.m_ixy, link->m_inertia.m_iyy, link->m_inertia.m_iyz,
                                link->m_inertia.m_ixz, link->m_inertia.m_iyz, link->m_inertia.m_izz);
                            btScalar threshold = 1.0e-6;
                            int numIterations = 30;
                            inertiaTensor.diagonalize(linkInertiaBasis, threshold, numIterations);
                            principalInertiaX = inertiaTensor[0][0];
                            principalInertiaY = inertiaTensor[1][1];
                            principalInertiaZ = inertiaTensor[2][2];
                        }
                    }
                    mass = linkMass;
                    if (principalInertiaX < 0 || principalInertiaX > (principalInertiaY + principalInertiaZ) || principalInertiaY < 0 || principalInertiaY > (principalInertiaX + principalInertiaZ) || principalInertiaZ < 0 || principalInertiaZ > (principalInertiaX + principalInertiaY)) {
                        b3Warning("Bad inertia tensor properties, setting inertia to zero for link: %s\n", link->m_name.c_str());
                        principalInertiaX = 0.f;
                        principalInertiaY = 0.f;
                        principalInertiaZ = 0.f;
                        linkInertiaBasis.setIdentity();
                    }
                    localInertiaDiagonal.setValue(principalInertiaX, principalInertiaY, principalInertiaZ);
                    inertialFrame.setOrigin(link->m_inertia.m_linkLocalFrame.getOrigin());
                    inertialFrame.setBasis(link->m_inertia.m_linkLocalFrame.getBasis() * linkInertiaBasis);
                }
                else {
                    mass = 1.f;
                    localInertiaDiagonal.setValue(1, 1, 1);
                    inertialFrame.setIdentity();
                }
            }
            else {
                //the link->m_inertia is NOT necessarily aligned with the inertial frame
                //so an additional transform might need to be computed
                UrdfLink* const* linkPtr = m_data->m_urdfParser.getModel().m_links.getAtIndex(urdfLinkIndex);

                btAssert(linkPtr);
                if (linkPtr) {
                    UrdfLink* link = *linkPtr;
                    btScalar linkMass;
                    if (link->m_parentJoint == 0 && m_data->m_urdfParser.getModel().m_overrideFixedBase) {
                        linkMass = 0.f;
                    }
                    else {
                        linkMass = link->m_inertia.m_mass;
                    }
                    mass = linkMass;
                    localInertiaDiagonal.setValue(0, 0, 0);
                    inertialFrame.setOrigin(link->m_inertia.m_linkLocalFrame.getOrigin());
                    inertialFrame.setBasis(link->m_inertia.m_linkLocalFrame.getBasis());
                }
                else {
                    mass = 1.f;
                    localInertiaDiagonal.setValue(1, 1, 1);
                    inertialFrame.setIdentity();
                }
            }
        }

        bool ImporterURDF::getJointInfo2(int urdfLinkIndex, btTransform& parent2joint, btTransform& linkTransformInWorld, btVector3& jointAxisInJointSpace, int& jointType, btScalar& jointLowerLimit, btScalar& jointUpperLimit, btScalar& jointDamping, btScalar& jointFriction, btScalar& jointMaxForce, btScalar& jointMaxVelocity) const
        {
            jointLowerLimit = 0.f;
            jointUpperLimit = 0.f;
            jointDamping = 0.f;
            jointFriction = 0.f;
            jointMaxForce = 0.f;
            jointMaxVelocity = 0.f;

            UrdfLink* const* linkPtr = m_data->m_urdfParser.getModel().m_links.getAtIndex(urdfLinkIndex);
            btAssert(linkPtr);
            if (linkPtr) {
                UrdfLink* link = *linkPtr;
                linkTransformInWorld = link->m_linkTransformInWorld;

                if (link->m_parentJoint) {
                    UrdfJoint* pj = link->m_parentJoint;
                    parent2joint = pj->m_parentLinkToJointTransform;
                    jointType = pj->m_type;
                    jointAxisInJointSpace = pj->m_localJointAxis;
                    jointLowerLimit = pj->m_lowerLimit;
                    jointUpperLimit = pj->m_upperLimit;
                    jointDamping = pj->m_jointDamping;
                    jointFriction = pj->m_jointFriction;
                    jointMaxForce = pj->m_effortLimit;
                    jointMaxVelocity = pj->m_velocityLimit;
                    return true;
                }
                else {
                    parent2joint.setIdentity();
                    return false;
                }
            }

            return false;
        };

        std::string ImporterURDF::getLinkName(int linkIndex) const
        {
            UrdfLink* const* linkPtr = m_data->m_urdfParser.getModel().m_links.getAtIndex(linkIndex);
            btAssert(linkPtr);
            if (linkPtr) {
                UrdfLink* link = *linkPtr;
                return link->m_name;
            }
            return "";
        }

        class btCompoundShape* ImporterURDF::convertLinkCollisionShapes(int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame) const
        {
            btCompoundShape* compoundShape = new btCompoundShape();
            m_data->m_allocatedCollisionShapes.push_back(compoundShape);

            compoundShape->setMargin(gUrdfDefaultCollisionMargin);
            UrdfLink* const* linkPtr = m_data->m_urdfParser.getModel().m_links.getAtIndex(linkIndex);
            btAssert(linkPtr);
            if (linkPtr) {
                UrdfLink* link = *linkPtr;

                for (int v = 0; v < link->m_collisionArray.size(); v++) {
                    const UrdfCollision& col = link->m_collisionArray[v];
                    btCollisionShape* childShape = convertURDFToCollisionShape(&col, pathPrefix);
                    if (childShape) {
                        m_data->m_allocatedCollisionShapes.push_back(childShape);
                        if (childShape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE) {
                            btCompoundShape* compound = (btCompoundShape*)childShape;
                            for (int i = 0; i < compound->getNumChildShapes(); i++) {
                                m_data->m_allocatedCollisionShapes.push_back(compound->getChildShape(i));
                            }
                        }

                        btTransform childTrans = col.m_linkLocalFrame;

                        compoundShape->addChildShape(localInertiaFrame.inverse() * childTrans, childShape);
                    }
                }
            }

            return compoundShape;
        }

        btCollisionShape* ImporterURDF::createConvexHullFromShapes(const tinyobj::attrib_t& attribute, std::vector<tinyobj::shape_t>& shapes, const btVector3& geomScale, int flags)
        {
            B3_PROFILE("createConvexHullFromShapes");
            btCompoundShape* compound = new btCompoundShape();
            compound->setMargin(gUrdfDefaultCollisionMargin);

            btTransform identity;
            identity.setIdentity();

            for (int s = 0; s < (int)shapes.size(); s++) {
                btConvexHullShape* convexHull = new btConvexHullShape();
                convexHull->setMargin(gUrdfDefaultCollisionMargin);
                tinyobj::shape_t& shape = shapes[s];
                int faceCount = shape.mesh.indices.size();

                for (int f = 0; f < faceCount; f += 3) {
                    btVector3 pt;
                    pt.setValue(attribute.vertices[3 * shape.mesh.indices[f + 0].vertex_index + 0],
                        attribute.vertices[3 * shape.mesh.indices[f + 0].vertex_index + 1],
                        attribute.vertices[3 * shape.mesh.indices[f + 0].vertex_index + 2]);

                    convexHull->addPoint(pt * geomScale, false);

                    pt.setValue(attribute.vertices[3 * shape.mesh.indices[f + 1].vertex_index + 0],
                        attribute.vertices[3 * shape.mesh.indices[f + 1].vertex_index + 1],
                        attribute.vertices[3 * shape.mesh.indices[f + 1].vertex_index + 2]);
                    convexHull->addPoint(pt * geomScale, false);

                    pt.setValue(attribute.vertices[3 * shape.mesh.indices[f + 2].vertex_index + 0],
                        attribute.vertices[3 * shape.mesh.indices[f + 2].vertex_index + 1],
                        attribute.vertices[3 * shape.mesh.indices[f + 2].vertex_index + 2]);
                    convexHull->addPoint(pt * geomScale, false);
                }

                convexHull->recalcLocalAabb();
                convexHull->optimizeConvexHull();
                if (flags & CUF_INITIALIZE_SAT_FEATURES) {
                    convexHull->initializePolyhedralFeatures();
                }

                compound->addChildShape(identity, convexHull);
            }

            return compound;
        }

        btCollisionShape* ImporterURDF::convertURDFToCollisionShape(const UrdfCollision* collision, const char* urdfPathPrefix) const
        {
            BT_PROFILE("convertURDFToCollisionShape");

            btCollisionShape* shape = 0;

            switch (collision->m_geometry.m_type) {
            case URDF_GEOM_PLANE: {
                btVector3 planeNormal = collision->m_geometry.m_planeNormal;
                btScalar planeConstant = 0; //not available?
                btStaticPlaneShape* plane = new btStaticPlaneShape(planeNormal, planeConstant);
                shape = plane;
                shape->setMargin(gUrdfDefaultCollisionMargin);
                break;
            }
            case URDF_GEOM_CAPSULE: {
                btScalar radius = collision->m_geometry.m_capsuleRadius;
                btScalar height = collision->m_geometry.m_capsuleHeight;
                btCapsuleShapeZ* capsuleShape = new btCapsuleShapeZ(radius, height);
                shape = capsuleShape;
                shape->setMargin(gUrdfDefaultCollisionMargin);
                break;
            }

            case URDF_GEOM_CYLINDER: {
                btScalar cylRadius = collision->m_geometry.m_capsuleRadius;
                btScalar cylHalfLength = 0.5 * collision->m_geometry.m_capsuleHeight;
                if (m_data->m_flags & CUF_USE_IMPLICIT_CYLINDER) {
                    btVector3 halfExtents(cylRadius, cylRadius, cylHalfLength);
                    btCylinderShapeZ* cylZShape = new btCylinderShapeZ(halfExtents);
                    shape = cylZShape;
                    shape->setMargin(gUrdfDefaultCollisionMargin);
                }
                else {
                    btAlignedObjectArray<btVector3> vertices;
                    //int numVerts = sizeof(barrel_vertices)/(9*sizeof(float));
                    int numSteps = 32;
                    for (int i = 0; i < numSteps; i++) {
                        btVector3 vert(cylRadius * btSin(SIMD_2_PI * (float(i) / numSteps)), cylRadius * btCos(SIMD_2_PI * (float(i) / numSteps)), cylHalfLength);
                        vertices.push_back(vert);
                        vert[2] = -cylHalfLength;
                        vertices.push_back(vert);
                    }
                    btConvexHullShape* cylZShape = new btConvexHullShape(&vertices[0].x(), vertices.size(), sizeof(btVector3));
                    cylZShape->setMargin(gUrdfDefaultCollisionMargin);
                    cylZShape->recalcLocalAabb();
                    if (m_data->m_flags & CUF_INITIALIZE_SAT_FEATURES) {
                        cylZShape->initializePolyhedralFeatures();
                    }
                    cylZShape->optimizeConvexHull();
                    shape = cylZShape;
                }

                break;
            }
            case URDF_GEOM_BOX: {
                btVector3 extents = collision->m_geometry.m_boxSize;
                btBoxShape* boxShape = new btBoxShape(extents * 0.5f);
                //btConvexShape* boxShape = new btConeShapeX(extents[2]*0.5,extents[0]*0.5);
                if (m_data->m_flags & CUF_INITIALIZE_SAT_FEATURES) {
                    boxShape->initializePolyhedralFeatures();
                }
                shape = boxShape;
                shape->setMargin(gUrdfDefaultCollisionMargin);
                break;
            }
            case URDF_GEOM_SPHERE: {
                btScalar radius = collision->m_geometry.m_sphereRadius;
                btSphereShape* sphereShape = new btSphereShape(radius);
                shape = sphereShape;
                shape->setMargin(gUrdfDefaultCollisionMargin);
                break;
            }
            case URDF_GEOM_CDF: {
                char relativeFileName[1024];
                char pathPrefix[1024];
                pathPrefix[0] = 0;
                if (m_data->m_fileIO->findResourcePath(collision->m_geometry.m_meshFileName.c_str(), relativeFileName, 1024)) {
                    b3FileUtils::extractPath(relativeFileName, pathPrefix, 1024);

                    btAlignedObjectArray<char> sdfData;
                    {
                        std::streampos fsize = 0;
                        std::ifstream file(relativeFileName, std::ios::binary);
                        if (file.good()) {
                            fsize = file.tellg();
                            file.seekg(0, std::ios::end);
                            fsize = file.tellg() - fsize;
                            file.seekg(0, std::ios::beg);
                            sdfData.resize(fsize);
                            int bytesRead = file.rdbuf()->sgetn(&sdfData[0], fsize);
                            btAssert(bytesRead == fsize);
                            file.close();
                        }
                    }

                    if (sdfData.size()) {
                        btSdfCollisionShape* sdfShape = new btSdfCollisionShape();
                        bool valid = sdfShape->initializeSDF(&sdfData[0], sdfData.size());
                        btAssert(valid);

                        if (valid) {
                            shape = sdfShape;
                        }
                        else {
                            delete sdfShape;
                        }
                    }
                }
                break;
            }
            case URDF_GEOM_MESH: {
                GLInstanceGraphicsShape* glmesh = 0;
                switch (collision->m_geometry.m_meshFileType) {
                case UrdfGeometry::FILE_OBJ:
                    if (collision->m_flags & URDF_FORCE_CONCAVE_TRIMESH) {
                        char relativeFileName[1024];
                        char pathPrefix[1024];
                        pathPrefix[0] = 0;
                        if (m_data->m_fileIO->findResourcePath(collision->m_geometry.m_meshFileName.c_str(), relativeFileName, 1024)) {
                            b3FileUtils::extractPath(relativeFileName, pathPrefix, 1024);
                        }
                        glmesh = LoadMeshFromObj(collision->m_geometry.m_meshFileName.c_str(), pathPrefix, m_data->m_fileIO);
                    }
                    else {
                        std::vector<tinyobj::shape_t> shapes;
                        tinyobj::attrib_t attribute;
                        std::string err = tinyobj::LoadObj(attribute, shapes, collision->m_geometry.m_meshFileName.c_str(), "", m_data->m_fileIO);
                        //create a convex hull for each shape, and store it in a btCompoundShape
                        shape = createConvexHullFromShapes(attribute, shapes, collision->m_geometry.m_meshScale, m_data->m_flags);
                        m_data->m_bulletCollisionShape2UrdfCollision.insert(shape, *collision);
                        return shape;
                    }
                    break;

                case UrdfGeometry::FILE_STL:
                    glmesh = LoadMeshFromSTL(collision->m_geometry.m_meshFileName.c_str(), m_data->m_fileIO);
                    break;

                case UrdfGeometry::FILE_COLLADA: {
                    btAlignedObjectArray<GLInstanceGraphicsShape> visualShapes;
                    btAlignedObjectArray<ColladaGraphicsInstance> visualShapeInstances;
                    btTransform upAxisTrans;
                    upAxisTrans.setIdentity();
                    float unitMeterScaling = 1;
                    LoadMeshFromCollada(collision->m_geometry.m_meshFileName.c_str(), visualShapes, visualShapeInstances, upAxisTrans, unitMeterScaling, 2, m_data->m_fileIO);

                    glmesh = new GLInstanceGraphicsShape;
                    glmesh->m_indices = new b3AlignedObjectArray<int>();
                    glmesh->m_vertices = new b3AlignedObjectArray<GLInstanceVertex>();

                    for (int i = 0; i < visualShapeInstances.size(); i++) {
                        ColladaGraphicsInstance* instance = &visualShapeInstances[i];
                        GLInstanceGraphicsShape* gfxShape = &visualShapes[instance->m_shapeIndex];

                        b3AlignedObjectArray<GLInstanceVertex> verts;
                        verts.resize(gfxShape->m_vertices->size());

                        int baseIndex = glmesh->m_vertices->size();

                        for (int i = 0; i < gfxShape->m_vertices->size(); i++) {
                            verts[i].normal[0] = gfxShape->m_vertices->at(i).normal[0];
                            verts[i].normal[1] = gfxShape->m_vertices->at(i).normal[1];
                            verts[i].normal[2] = gfxShape->m_vertices->at(i).normal[2];
                            verts[i].uv[0] = gfxShape->m_vertices->at(i).uv[0];
                            verts[i].uv[1] = gfxShape->m_vertices->at(i).uv[1];
                            verts[i].xyzw[0] = gfxShape->m_vertices->at(i).xyzw[0];
                            verts[i].xyzw[1] = gfxShape->m_vertices->at(i).xyzw[1];
                            verts[i].xyzw[2] = gfxShape->m_vertices->at(i).xyzw[2];
                            verts[i].xyzw[3] = gfxShape->m_vertices->at(i).xyzw[3];
                        }

                        int curNumIndices = glmesh->m_indices->size();
                        int additionalIndices = gfxShape->m_indices->size();
                        glmesh->m_indices->resize(curNumIndices + additionalIndices);
                        for (int k = 0; k < additionalIndices; k++) {
                            glmesh->m_indices->at(curNumIndices + k) = gfxShape->m_indices->at(k) + baseIndex;
                        }

                        //compensate upAxisTrans and unitMeterScaling here
                        utils::btMatrix4x4 upAxisMat;
                        upAxisMat.setIdentity();
                        //upAxisMat.setPureRotation(upAxisTrans.getRotation());
                        utils::btMatrix4x4 unitMeterScalingMat;
                        unitMeterScalingMat.setPureScaling(btVector3(unitMeterScaling, unitMeterScaling, unitMeterScaling));
                        utils::btMatrix4x4 worldMat = unitMeterScalingMat * instance->m_worldTransform * upAxisMat;
                        //btMatrix4x4 worldMat = instance->m_worldTransform;
                        int curNumVertices = glmesh->m_vertices->size();
                        int additionalVertices = verts.size();
                        glmesh->m_vertices->reserve(curNumVertices + additionalVertices);

                        for (int v = 0; v < verts.size(); v++) {
                            btVector3 pos(verts[v].xyzw[0], verts[v].xyzw[1], verts[v].xyzw[2]);
                            pos = worldMat * pos;
                            verts[v].xyzw[0] = float(pos[0]);
                            verts[v].xyzw[1] = float(pos[1]);
                            verts[v].xyzw[2] = float(pos[2]);
                            glmesh->m_vertices->push_back(verts[v]);
                        }
                    }
                    glmesh->m_numIndices = glmesh->m_indices->size();
                    glmesh->m_numvertices = glmesh->m_vertices->size();
                    //glmesh = LoadMeshFromCollada(success.c_str());
                    break;
                }
                }

                if (!glmesh || glmesh->m_numvertices <= 0) {
                    b3Warning("%s: cannot extract mesh from '%s'\n", urdfPathPrefix, collision->m_geometry.m_meshFileName.c_str());
                    delete glmesh;
                    break;
                }

                btAlignedObjectArray<btVector3> convertedVerts;
                convertedVerts.reserve(glmesh->m_numvertices);
                for (int i = 0; i < glmesh->m_numvertices; i++) {
                    convertedVerts.push_back(btVector3(
                        glmesh->m_vertices->at(i).xyzw[0] * collision->m_geometry.m_meshScale[0],
                        glmesh->m_vertices->at(i).xyzw[1] * collision->m_geometry.m_meshScale[1],
                        glmesh->m_vertices->at(i).xyzw[2] * collision->m_geometry.m_meshScale[2]));
                }

                if (collision->m_flags & URDF_FORCE_CONCAVE_TRIMESH) {
                    BT_PROFILE("convert trimesh");
                    btTriangleMesh* meshInterface = new btTriangleMesh();
                    m_data->m_allocatedMeshInterfaces.push_back(meshInterface);
                    {
                        BT_PROFILE("convert vertices");

                        for (int i = 0; i < glmesh->m_numIndices / 3; i++) {
                            const btVector3& v0 = convertedVerts[glmesh->m_indices->at(i * 3)];
                            const btVector3& v1 = convertedVerts[glmesh->m_indices->at(i * 3 + 1)];
                            const btVector3& v2 = convertedVerts[glmesh->m_indices->at(i * 3 + 2)];
                            meshInterface->addTriangle(v0, v1, v2);
                        }
                    }
                    {
                        BT_PROFILE("create btBvhTriangleMeshShape");
                        btBvhTriangleMeshShape* trimesh = new btBvhTriangleMeshShape(meshInterface, true, true);
                        //trimesh->setLocalScaling(collision->m_geometry.m_meshScale);
                        shape = trimesh;
                    }
                }
                else {
                    BT_PROFILE("convert btConvexHullShape");
                    btConvexHullShape* convexHull = new btConvexHullShape(&convertedVerts[0].getX(), convertedVerts.size(), sizeof(btVector3));
                    convexHull->optimizeConvexHull();
                    if (m_data->m_flags & CUF_INITIALIZE_SAT_FEATURES) {
                        convexHull->initializePolyhedralFeatures();
                    }
                    convexHull->setMargin(gUrdfDefaultCollisionMargin);
                    convexHull->recalcLocalAabb();
                    //convexHull->setLocalScaling(collision->m_geometry.m_meshScale);
                    shape = convexHull;
                }

                delete glmesh;
                break;
            } // mesh case

            default:
                b3Warning("Error: unknown collision geometry type %i\n", collision->m_geometry.m_type);
            }
            if (shape && collision->m_geometry.m_type == URDF_GEOM_MESH) {
                m_data->m_bulletCollisionShape2UrdfCollision.insert(shape, *collision);
            }
            return shape;
        }

        void ImporterURDF::setLinkColor2(int linkIndex, struct UrdfMaterialColor& matCol) const
        {
            m_data->m_linkColors.insert(linkIndex, matCol);
        }

        bool ImporterURDF::getLinkColor2(int linkIndex, UrdfMaterialColor& matCol) const
        {
            UrdfMaterialColor* matColPtr = m_data->m_linkColors[linkIndex];
            if (matColPtr) {
                matCol = *matColPtr;
                return true;
            }
            return false;
        }

        bool ImporterURDF::getLinkContactInfo(int urdflinkIndex, URDFLinkContactInfo& contactInfo) const
        {
            UrdfLink* const* linkPtr = m_data->m_urdfParser.getModel().m_links.getAtIndex(urdflinkIndex);
            if (linkPtr) {
                const UrdfLink* link = *linkPtr;
                contactInfo = link->m_contactInfo;
                return true;
            }
            return false;
        }

        int ImporterURDF::getCollisionGroupAndMask(int linkIndex, int& colGroup, int& colMask) const
        {
            int result = 0;
            UrdfLink* const* linkPtr = m_data->m_urdfParser.getModel().m_links.getAtIndex(linkIndex);
            btAssert(linkPtr);
            if (linkPtr) {
                UrdfLink* link = *linkPtr;
                for (int v = 0; v < link->m_collisionArray.size(); v++) {
                    const UrdfCollision& col = link->m_collisionArray[v];
                    if (col.m_flags & URDF_HAS_COLLISION_GROUP) {
                        colGroup = col.m_collisionGroup;
                        result |= URDF_HAS_COLLISION_GROUP;
                    }
                    if (col.m_flags & URDF_HAS_COLLISION_MASK) {
                        colMask = col.m_collisionMask;
                        result |= URDF_HAS_COLLISION_MASK;
                    }
                }
            }
            return result;
        }

        const char* ImporterURDF::getPathPrefix()
        {
            return m_data->m_pathPrefix;
        }

        int ImporterURDF::getNumAllocatedCollisionShapes() const
        {
            return m_data->m_allocatedCollisionShapes.size();
        }

        btCollisionShape* ImporterURDF::getAllocatedCollisionShape(int index)
        {
            return m_data->m_allocatedCollisionShapes[index];
        }
    } // namespace importers
} // namespace robot_bullet