#ifndef ROBOT_BULLET_IMPORTER_URDF
#define ROBOT_BULLET_IMPORTER_URDF

#include <fstream>
#include <string>
#include <vector>

#include <Bullet3Common/b3FileUtils.h>
#include <btBulletCollisionCommon.h>

#include <BulletCollision/CollisionShapes/btSdfCollisionShape.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>

#include "robot_bullet/importers/ParserURDF.hpp"
#include "robot_bullet/interfaces/DefaultFileIO.hpp"
#include "robot_bullet/thirdparty/tiny_obj_loader.h"

namespace robot_bullet {
    namespace importers {
        ATTRIBUTE_ALIGNED16(struct)
        BulletURDFInternalData
        {
            BT_DECLARE_ALIGNED_ALLOCATOR();

            interfaces::DefaultFileIO m_defaultFileIO;

            ParserURDF m_urdfParser;

            interfaces::FileIOInterface* m_fileIO;

            std::string m_sourceFile;
            char m_pathPrefix[1024];
            int m_bodyId;
            btHashMap<btHashInt, UrdfMaterialColor> m_linkColors;
            btAlignedObjectArray<btCollisionShape*> m_allocatedCollisionShapes;
            btAlignedObjectArray<int> m_allocatedTextures;
            mutable btAlignedObjectArray<btTriangleMesh*> m_allocatedMeshInterfaces;
            btHashMap<btHashPtr, UrdfCollision> m_bulletCollisionShape2UrdfCollision;

            int m_flags;

            void setSourceFile(const std::string& relativeFileName, const std::string& prefix)
            {
                m_sourceFile = relativeFileName;
                m_urdfParser.setSourceFile(relativeFileName);
                strncpy(m_pathPrefix, prefix.c_str(), sizeof(m_pathPrefix));
                m_pathPrefix[sizeof(m_pathPrefix) - 1] = 0; // required, strncpy doesn't write zero on overflow
            }

            BulletURDFInternalData(interfaces::FileIOInterface * fileIO)
                : m_urdfParser(fileIO ? fileIO : &m_defaultFileIO),
                  m_fileIO(fileIO ? fileIO : &m_defaultFileIO)
            {
                m_pathPrefix[0] = 0;
                m_flags = 0;
            }

            void setGlobalScaling(btScalar scaling)
            {
                m_urdfParser.setGlobalScaling(scaling);
            }
        };

        struct BulletErrorLogger : public ErrorLogger {
            int m_numErrors;
            int m_numWarnings;

            BulletErrorLogger()
                : m_numErrors(0),
                  m_numWarnings(0)
            {
            }
            virtual void reportError(const char* error)
            {
                m_numErrors++;
                b3Error(error);
            }
            virtual void reportWarning(const char* warning)
            {
                m_numWarnings++;
                b3Warning(warning);
            }

            virtual void printMessage(const char* msg)
            {
                b3Printf(msg);
            }
        };

        class ImporterURDF {
        public:
            ImporterURDF(interfaces::FileIOInterface* fileIO = 0, double globalScaling = 1, int flags = 0)
            {
                m_data = new BulletURDFInternalData(fileIO);
                m_data->setGlobalScaling(globalScaling);
                m_data->m_flags = flags;
            }

            ~ImporterURDF() {}

            bool loadURDF(const char* fileName, bool forceFixedBase = false);

            int getRootLinkIndex() const;

            void getLinkChildIndices(int linkIndex, btAlignedObjectArray<int>& childLinkIndices) const;

            void getMassAndInertia(int urdfLinkIndex, btScalar& mass, btVector3& localInertiaDiagonal, btTransform& inertialFrame, int flags) const;

            bool getJointInfo2(int urdfLinkIndex, btTransform& parent2joint, btTransform& linkTransformInWorld, btVector3& jointAxisInJointSpace, int& jointType, btScalar& jointLowerLimit, btScalar& jointUpperLimit, btScalar& jointDamping, btScalar& jointFriction, btScalar& jointMaxForce, btScalar& jointMaxVelocity) const;

            std::string getLinkName(int linkIndex) const;

            btCompoundShape* convertLinkCollisionShapes(int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame) const;

            static btCollisionShape* createConvexHullFromShapes(const tinyobj::attrib_t& attribute, std::vector<tinyobj::shape_t>& shapes, const btVector3& geomScale, int flags);

            btCollisionShape* convertURDFToCollisionShape(const UrdfCollision* collision, const char* urdfPathPrefix) const;

            void setLinkColor2(int linkIndex, struct UrdfMaterialColor& matCol) const;

            bool getLinkColor2(int linkIndex, UrdfMaterialColor& matCol) const;

            bool getLinkContactInfo(int urdflinkIndex, URDFLinkContactInfo& contactInfo) const;

            int getCollisionGroupAndMask(int linkIndex, int& colGroup, int& colMask) const;

            const char* getPathPrefix();

            int getNumAllocatedCollisionShapes() const;

            btCollisionShape* getAllocatedCollisionShape(int index);

            std::vector<std::vector<std::string>> getLinkMeshes();

        protected:
            BulletURDFInternalData* m_data;
        };
    } // namespace importers
} // namespace robot_bullet

#endif // ROBOT_BULLET_IMPORTER_URDF