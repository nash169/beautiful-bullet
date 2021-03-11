#ifndef ROBOT_BULLET_INTERFACES_FILE_IO_INTERFACE_H
#define ROBOT_BULLET_INTERFACES_FILE_IO_INTERFACE_H

namespace robot_bullet {
    namespace interfaces {
        struct FileIOInterface {
            int m_fileIOType;
            const char* m_pathPrefix;

            FileIOInterface(int fileIOType, const char* pathPrefix)
                : m_fileIOType(fileIOType),
                  m_pathPrefix(pathPrefix)
            {
            }

            virtual ~FileIOInterface()
            {
            }
            virtual int fileOpen(const char* fileName, const char* mode) = 0;
            virtual int fileRead(int fileHandle, char* destBuffer, int numBytes) = 0;
            virtual int fileWrite(int fileHandle, const char* sourceBuffer, int numBytes) = 0;
            virtual void fileClose(int fileHandle) = 0;
            virtual bool findResourcePath(const char* fileName, char* resourcePathOut, int resourcePathMaxNumBytes) = 0;
            virtual char* readLine(int fileHandle, char* destBuffer, int numBytes) = 0;
            virtual int getFileSize(int fileHandle) = 0;
            virtual void enableFileCaching(bool enable) = 0;
        };
    } // namespace interfaces
} // namespace robot_bullet

#endif // ROBOT_BULLET_INTERFACES_FILE_IO_INTERFACE_H