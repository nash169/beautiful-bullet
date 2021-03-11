#ifndef ROBOT_BULLET_UTILS_RESOURCE_PATH_H
#define ROBOT_BULLET_UTILS_RESOURCE_PATH_H

#include <string>

#include <Bullet3Common/b3FileUtils.h>
#include <Bullet3Common/b3Logging.h>

#ifdef __APPLE__
#include <mach-o/dyld.h> /* _NSGetExecutablePath */
#else
#ifdef _WIN32
#include <windows.h>
#else
//not Mac, not Windows, let's cross the fingers it is Linux :-)
#include <unistd.h>
#endif
#endif

#define B3_MAX_EXE_PATH_LEN 4096

namespace robot_bullet {
    namespace utils {
        typedef bool (*PFN_FIND_FILE)(void* userPointer, const char* orgFileName, char* relativeFileName, int maxRelativeFileNameMaxLen);

        class ResourcePath {
        public:
            static int getExePath(char* path, int maxPathLenInBytes);
            static int findResourcePath(const char* resourceName, char* resourcePathOut, int resourcePathMaxNumBytes, PFN_FIND_FILE findFile, void* userPointer = 0);
            static void setAdditionalSearchPath(const char* path);
        };
    } // namespace utils
} // namespace robot_bullet
#endif // ROBOT_BULLET_UTILS
