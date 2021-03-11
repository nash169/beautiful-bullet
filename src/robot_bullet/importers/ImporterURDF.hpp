#ifndef ROBOT_BULLET_IMPORTER_URDF
#define ROBOT_BULLET_IMPORTER_URDF

#include <iostream>
#include <string>

#include <Bullet3Common/b3FileUtils.h>
#include <btBulletCollisionCommon.h>

#include "robot_bullet/importers/ParserURDF.hpp"
#include "robot_bullet/interfaces/DefaultFileIO.hpp"

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

            bool loadURDF(const char* fileName, bool forceFixedBase = false)
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

        protected:
            BulletURDFInternalData* m_data;
        };
    } // namespace importers
} // namespace robot_bullet

#endif // ROBOT_BULLET_IMPORTER_URDF