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

#ifndef BEAUTIFUL_BULLET_UTILS_URDF_MESHSHAPE
#define BEAUTIFUL_BULLET_UTILS_URDF_MESHSHAPE

#include <iostream>

#include <BulletCollision/btBulletCollisionCommon.h>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <assimp/cimport.h>

namespace beautiful_bullet {
    namespace utils {

        class MeshShape {
        public:
            MeshShape()
            {
            }

            // const aiScene* loadMesh(const std::string& path)
            // {
            //     // // Create importer
            //     // Assimp::Importer importer;

            //     // Remove points and lines from the import.
            //     _importer.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_POINT | aiPrimitiveType_LINE);

            //     // Import the file.
            //     const aiScene* scene = _importer.ReadFile(path,
            //         aiProcess_GenNormals | aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType | aiProcess_OptimizeMeshes);

            //     // If succeeded, store the importer in the scene to keep it alive. This is
            //     // necessary because the importer owns the memory that it allocates.
            //     if (!scene) {
            //         dtwarn << "[MeshShape::loadMesh] Failed loading mesh '" << path << "'.\n";
            //         return nullptr;
            //     }

            //     return scene;
            // }

            // Assimp basic usage: https://assimp-docs.readthedocs.io/en/latest/usage/use_the_lib.html
            const aiScene* loadMesh(const std::string& path)
            {
                // Remove points and lines from the import.
                aiPropertyStore* propertyStore = aiCreatePropertyStore();
                aiSetImportPropertyInteger(
                    propertyStore,
                    AI_CONFIG_PP_SBP_REMOVE,
                    aiPrimitiveType_POINT | aiPrimitiveType_LINE);

                // Import the file.
                const aiScene* scene = aiImportFileExWithProperties(
                    path.c_str(),
                    aiProcess_GenNormals | aiProcess_Triangulate
                        | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType
                        | aiProcess_OptimizeMeshes,
                    NULL,
                    propertyStore);

                // If succeeded, store the importer in the scene to keep it alive. This is
                // necessary because the importer owns the memory that it allocates.
                if (!scene) {
                    std::cerr << "[MeshShape::loadMesh] Failed loading mesh '" << path << "'.\n";
                    aiReleasePropertyStore(propertyStore);
                    return nullptr;
                }

                // Assimp rotates collada files such that the up-axis (specified in the
                // collada file) aligns with assimp's y-axis. Here we are reverting this
                // rotation. We are only catching files with the .dae file ending here. We
                // might miss files with an .xml file ending, which would need to be looked
                // into to figure out whether they are collada files.
                std::string extension;
                const std::size_t extensionIndex = path.find_last_of('.');
                if (extensionIndex != std::string::npos)
                    extension = path.substr(extensionIndex);

                std::transform(
                    std::begin(extension),
                    std::end(extension),
                    std::begin(extension),
                    ::tolower);

                if (extension == ".dae" || extension == ".zae")
                    scene->mRootNode->mTransformation = aiMatrix4x4();

                // Finally, pre-transform the vertices. We can't do this as part of the
                // import process, because we may have changed mTransformation above.
                scene = aiApplyPostProcessing(scene, aiProcess_PreTransformVertices);
                if (!scene)
                    std::cerr << "[MeshShape::loadMesh] Failed pre-transforming vertices.\n";

                aiReleasePropertyStore(propertyStore);

                return scene;
            }

        protected:
            // Assimp::Importer _importer;
        };
    } // namespace utils
} // namespace beautiful_bullet
#endif // BEAUTIFUL_BULLET_UTILS_URDF_MESHSHAPE