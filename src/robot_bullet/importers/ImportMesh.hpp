#ifndef ROBOT_BULLET_IMPORT_MESH
#define ROBOT_BULLET_IMPORT_MESH

#include <string.h> //memcpy

#include <Bullet3Common/b3AlignedObjectArray.h>
#include <Bullet3Common/b3HashMap.h>
#include <Bullet3Common/b3MinMax.h>

#include <LinearMath/btAlignedObjectArray.h>
#include <LinearMath/btHashMap.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btVector3.h>

#include "robot_bullet/interfaces/FileIOInterface.hpp"
#include "robot_bullet/thirdparty/tiny_obj_loader.h"
#include "robot_bullet/thirdparty/tinyxml2.h"
#include "robot_bullet/utils/btMatrix4x4.h"

#define MAX_VISUAL_SHAPES 512

namespace robot_bullet {
    namespace importers {
        using namespace tinyxml2;

        struct GLInstanceVertex {
            float xyzw[4];
            float normal[3];
            float uv[2];
        };

        struct GLInstanceGraphicsShape {
            b3AlignedObjectArray<GLInstanceVertex>* m_vertices;
            int m_numvertices;
            b3AlignedObjectArray<int>* m_indices;
            int m_numIndices;
            float m_scaling[4];

            GLInstanceGraphicsShape()
                : m_vertices(0),
                  m_indices(0)
            {
            }

            virtual ~GLInstanceGraphicsShape()
            {
                delete m_vertices;
                delete m_indices;
            }
        };

        struct CachedObjResult {
            std::string m_msg;
            std::vector<tinyobj::shape_t> m_shapes;
            tinyobj::attrib_t m_attribute;
        };

        static b3HashMap<b3HashString, CachedObjResult> gCachedObjResults;
        static int gEnableFileCaching = 1;

        int b3IsFileCachingEnabled()
        {
            return gEnableFileCaching;
        }

        void b3EnableFileCaching(int enable)
        {
            gEnableFileCaching = enable;
            if (enable == 0) {
                gCachedObjResults.clear();
            }
        }

        std::string LoadFromCachedOrFromObj(
            tinyobj::attrib_t& attribute,
            std::vector<tinyobj::shape_t>& shapes, // [output]
            const char* filename,
            const char* mtl_basepath,
            interfaces::FileIOInterface* fileIO)
        {
            CachedObjResult* resultPtr = gCachedObjResults[filename];
            if (resultPtr) {
                const CachedObjResult& result = *resultPtr;
                shapes = result.m_shapes;
                attribute = result.m_attribute;
                return result.m_msg;
            }

            std::string err = tinyobj::LoadObj(attribute, shapes, filename, mtl_basepath, fileIO);
            CachedObjResult result;
            result.m_msg = err;
            result.m_shapes = shapes;
            result.m_attribute = attribute;
            if (gEnableFileCaching) {
                gCachedObjResults.insert(filename, result);
            }
            return err;
        }

        GLInstanceGraphicsShape* btgCreateGraphicsShapeFromWavefrontObj(const tinyobj::attrib_t& attribute, std::vector<tinyobj::shape_t>& shapes, bool flatShading = false)
        {
            b3AlignedObjectArray<GLInstanceVertex>* vertices = new b3AlignedObjectArray<GLInstanceVertex>;
            {
                //		int numVertices = obj->vertexCount;
                //	int numIndices = 0;
                b3AlignedObjectArray<int>* indicesPtr = new b3AlignedObjectArray<int>;

                for (int s = 0; s < (int)shapes.size(); s++) {
                    tinyobj::shape_t& shape = shapes[s];
                    int faceCount = shape.mesh.indices.size();

                    for (int f = 0; f < faceCount; f += 3) {
                        //btVector3 normal(face.m_plane[0],face.m_plane[1],face.m_plane[2]);
                        if (1) {
                            btVector3 normal(0, 1, 0);
                            int vtxBaseIndex = vertices->size();

                            if (f < 0 && f >= int(shape.mesh.indices.size())) {
                                continue;
                            }

                            GLInstanceVertex vtx0;
                            tinyobj::index_t v_0 = shape.mesh.indices[f];
                            vtx0.xyzw[0] = attribute.vertices[3 * v_0.vertex_index];
                            vtx0.xyzw[1] = attribute.vertices[3 * v_0.vertex_index + 1];
                            vtx0.xyzw[2] = attribute.vertices[3 * v_0.vertex_index + 2];
                            vtx0.xyzw[3] = 0.f;

                            if (attribute.texcoords.size()) {
                                int uv0Index = 2 * v_0.texcoord_index;
                                int uv1Index = 2 * v_0.texcoord_index + 1;
                                if (uv0Index >= 0 && uv1Index >= 0 && (uv0Index < int(attribute.texcoords.size()) && (uv1Index < attribute.texcoords.size()))) {
                                    vtx0.uv[0] = attribute.texcoords[uv0Index];
                                    vtx0.uv[1] = attribute.texcoords[uv1Index];
                                }
                                else {
                                    //	b3Warning("obj texture coordinate out-of-range!");
                                    vtx0.uv[0] = 0;
                                    vtx0.uv[1] = 0;
                                }
                            }
                            else {
                                vtx0.uv[0] = 0.5;
                                vtx0.uv[1] = 0.5;
                            }

                            GLInstanceVertex vtx1;
                            tinyobj::index_t v_1 = shape.mesh.indices[f + 1];
                            vtx1.xyzw[0] = attribute.vertices[3 * v_1.vertex_index];
                            vtx1.xyzw[1] = attribute.vertices[3 * v_1.vertex_index + 1];
                            vtx1.xyzw[2] = attribute.vertices[3 * v_1.vertex_index + 2];
                            vtx1.xyzw[3] = 0.f;

                            if (attribute.texcoords.size()) {
                                int uv0Index = 2 * v_1.texcoord_index;
                                int uv1Index = 2 * v_1.texcoord_index + 1;
                                if (uv0Index >= 0 && uv1Index >= 0 && (uv0Index < attribute.texcoords.size()) && (uv1Index < attribute.texcoords.size())) {
                                    vtx1.uv[0] = attribute.texcoords[uv0Index];
                                    vtx1.uv[1] = attribute.texcoords[uv1Index];
                                }
                                else {
                                    //	b3Warning("obj texture coordinate out-of-range!");
                                    vtx1.uv[0] = 0;
                                    vtx1.uv[1] = 0;
                                }
                            }
                            else {
                                vtx1.uv[0] = 0.5f;
                                vtx1.uv[1] = 0.5f;
                            }

                            GLInstanceVertex vtx2;
                            tinyobj::index_t v_2 = shape.mesh.indices[f + 2];
                            vtx2.xyzw[0] = attribute.vertices[3 * v_2.vertex_index];
                            vtx2.xyzw[1] = attribute.vertices[3 * v_2.vertex_index + 1];
                            vtx2.xyzw[2] = attribute.vertices[3 * v_2.vertex_index + 2];
                            vtx2.xyzw[3] = 0.f;
                            if (attribute.texcoords.size()) {
                                int uv0Index = 2 * v_2.texcoord_index;
                                int uv1Index = 2 * v_2.texcoord_index + 1;

                                if (uv0Index >= 0 && uv1Index >= 0 && (uv0Index < attribute.texcoords.size()) && (uv1Index < attribute.texcoords.size())) {
                                    vtx2.uv[0] = attribute.texcoords[uv0Index];
                                    vtx2.uv[1] = attribute.texcoords[uv1Index];
                                }
                                else {
                                    //b3Warning("obj texture coordinate out-of-range!");
                                    vtx2.uv[0] = 0;
                                    vtx2.uv[1] = 0;
                                }
                            }
                            else {
                                vtx2.uv[0] = 0.5;
                                vtx2.uv[1] = 0.5;
                            }

                            btVector3 v0(vtx0.xyzw[0], vtx0.xyzw[1], vtx0.xyzw[2]);
                            btVector3 v1(vtx1.xyzw[0], vtx1.xyzw[1], vtx1.xyzw[2]);
                            btVector3 v2(vtx2.xyzw[0], vtx2.xyzw[1], vtx2.xyzw[2]);

                            unsigned int maxIndex = 0;
                            unsigned n0Index = shape.mesh.indices[f].normal_index;
                            unsigned n1Index = shape.mesh.indices[f + 1].normal_index;
                            unsigned n2Index = shape.mesh.indices[f + 2].normal_index;

                            maxIndex = b3Max(maxIndex, 3 * n0Index + 0);
                            maxIndex = b3Max(maxIndex, 3 * n0Index + 1);
                            maxIndex = b3Max(maxIndex, 3 * n0Index + 2);
                            maxIndex = b3Max(maxIndex, 3 * n1Index + 0);
                            maxIndex = b3Max(maxIndex, 3 * n1Index + 1);
                            maxIndex = b3Max(maxIndex, 3 * n1Index + 2);
                            maxIndex = b3Max(maxIndex, 3 * n2Index + 0);
                            maxIndex = b3Max(maxIndex, 3 * n2Index + 1);
                            maxIndex = b3Max(maxIndex, 3 * n2Index + 2);

                            bool hasNormals = (attribute.normals.size() && maxIndex < attribute.normals.size());

                            if (flatShading || !hasNormals) {
                                normal = (v1 - v0).cross(v2 - v0);
                                btScalar len2 = normal.length2();
                                //skip degenerate triangles
                                if (len2 > SIMD_EPSILON) {
                                    normal.normalize();
                                }
                                else {
                                    normal.setValue(0, 0, 0);
                                }
                                vtx0.normal[0] = normal[0];
                                vtx0.normal[1] = normal[1];
                                vtx0.normal[2] = normal[2];
                                vtx1.normal[0] = normal[0];
                                vtx1.normal[1] = normal[1];
                                vtx1.normal[2] = normal[2];
                                vtx2.normal[0] = normal[0];
                                vtx2.normal[1] = normal[1];
                                vtx2.normal[2] = normal[2];
                            }
                            else {
                                vtx0.normal[0] = attribute.normals[3 * n0Index + 0];
                                vtx0.normal[1] = attribute.normals[3 * n0Index + 1];
                                vtx0.normal[2] = attribute.normals[3 * n0Index + 2];
                                vtx1.normal[0] = attribute.normals[3 * n1Index + 0];
                                vtx1.normal[1] = attribute.normals[3 * n1Index + 1];
                                vtx1.normal[2] = attribute.normals[3 * n1Index + 2];
                                vtx2.normal[0] = attribute.normals[3 * n2Index + 0];
                                vtx2.normal[1] = attribute.normals[3 * n2Index + 1];
                                vtx2.normal[2] = attribute.normals[3 * n2Index + 2];
                            }
                            vertices->push_back(vtx0);
                            vertices->push_back(vtx1);
                            vertices->push_back(vtx2);
                            indicesPtr->push_back(vtxBaseIndex);
                            indicesPtr->push_back(vtxBaseIndex + 1);
                            indicesPtr->push_back(vtxBaseIndex + 2);
                        }
                    }
                }

                GLInstanceGraphicsShape* gfxShape = new GLInstanceGraphicsShape;
                gfxShape->m_vertices = vertices;
                gfxShape->m_numvertices = vertices->size();
                gfxShape->m_indices = indicesPtr;
                gfxShape->m_numIndices = indicesPtr->size();
                for (int i = 0; i < 4; i++)
                    gfxShape->m_scaling[i] = 1; //bake the scaling into the vertices
                return gfxShape;
            }
        }

        GLInstanceGraphicsShape* LoadMeshFromObj(const char* relativeFileName, const char* materialPrefixPath, interfaces::FileIOInterface* fileIO)
        {
            B3_PROFILE("LoadMeshFromObj");
            std::vector<tinyobj::shape_t> shapes;
            tinyobj::attrib_t attribute;
            {
                B3_PROFILE("tinyobj::LoadObj2");
                std::string err = LoadFromCachedOrFromObj(attribute, shapes, relativeFileName, materialPrefixPath, fileIO);
            }

            {
                B3_PROFILE("btgCreateGraphicsShapeFromWavefrontObj");
                GLInstanceGraphicsShape* gfxShape = btgCreateGraphicsShapeFromWavefrontObj(attribute, shapes);
                return gfxShape;
            }
        }

        struct MySTLTriangle {
            float normal[3];
            float vertex0[3];
            float vertex1[3];
            float vertex2[3];
        };

        static GLInstanceGraphicsShape* LoadMeshFromSTL(const char* relativeFileName, struct interfaces::FileIOInterface* fileIO)
        {
            GLInstanceGraphicsShape* shape = 0;

            int fileHandle = fileIO->fileOpen(relativeFileName, "rb");
            if (fileHandle >= 0) {
                int size = 0;
                size = fileIO->getFileSize(fileHandle);
                {
                    if (size >= 0) {
                        //b3Warning("Open STL file of %d bytes\n",size);
                        char* memoryBuffer = new char[size + 1];
                        int actualBytesRead = fileIO->fileRead(fileHandle, memoryBuffer, size);
                        if (actualBytesRead != size) {
                            b3Warning("Error reading from file %s", relativeFileName);
                        }
                        else {
                            int numTriangles = *(int*)&memoryBuffer[80];

                            if (numTriangles) {
                                {
                                    //perform a sanity check instead of crashing on invalid triangles/STL files
                                    int expectedBinaryFileSize = numTriangles * 50 + 84;
                                    if (expectedBinaryFileSize != size) {
                                        delete[] memoryBuffer;
                                        fileIO->fileClose(fileHandle);
                                        return 0;
                                    }
                                }
                                shape = new GLInstanceGraphicsShape;
                                //						b3AlignedObjectArray<GLInstanceVertex>*	m_vertices;
                                //						int				m_numvertices;
                                //						b3AlignedObjectArray<int>* 		m_indices;
                                //						int				m_numIndices;
                                //						float			m_scaling[4];
                                shape->m_scaling[0] = 1;
                                shape->m_scaling[1] = 1;
                                shape->m_scaling[2] = 1;
                                shape->m_scaling[3] = 1;
                                int index = 0;
                                shape->m_indices = new b3AlignedObjectArray<int>();
                                shape->m_vertices = new b3AlignedObjectArray<GLInstanceVertex>();
                                for (int i = 0; i < numTriangles; i++) {
                                    char* curPtr = &memoryBuffer[84 + i * 50];
                                    MySTLTriangle tmp;
                                    memcpy(&tmp, curPtr, sizeof(MySTLTriangle));

                                    GLInstanceVertex v0, v1, v2;
                                    v0.uv[0] = v1.uv[0] = v2.uv[0] = 0.5;
                                    v0.uv[1] = v1.uv[1] = v2.uv[1] = 0.5;
                                    for (int v = 0; v < 3; v++) {
                                        v0.xyzw[v] = tmp.vertex0[v];
                                        v1.xyzw[v] = tmp.vertex1[v];
                                        v2.xyzw[v] = tmp.vertex2[v];
                                        v0.normal[v] = v1.normal[v] = v2.normal[v] = tmp.normal[v];
                                    }
                                    v0.xyzw[3] = v1.xyzw[3] = v2.xyzw[3] = 0.f;

                                    shape->m_vertices->push_back(v0);
                                    shape->m_vertices->push_back(v1);
                                    shape->m_vertices->push_back(v2);

                                    shape->m_indices->push_back(index++);
                                    shape->m_indices->push_back(index++);
                                    shape->m_indices->push_back(index++);
                                }
                            }
                        }

                        delete[] memoryBuffer;
                    }
                }
                fileIO->fileClose(fileHandle);
            }
            if (shape) {
                shape->m_numIndices = shape->m_indices->size();
                shape->m_numvertices = shape->m_vertices->size();
            }
            return shape;
        }

        struct ColladaGraphicsInstance {
            ColladaGraphicsInstance()
                : m_shapeIndex(-1)
            {
                m_worldTransform.setIdentity();
            }
            utils::btMatrix4x4 m_worldTransform;
            int m_shapeIndex; //could be index into array of GLInstanceGraphicsShape
            float m_color[4];
        };

        struct VertexSource {
            std::string m_positionArrayId;
            std::string m_normalArrayId;
        };

        struct TokenFloatArray {
            btAlignedObjectArray<float>& m_values;
            TokenFloatArray(btAlignedObjectArray<float>& floatArray)
                : m_values(floatArray)
            {
            }
            inline void add(const char* token)
            {
                float v = atof(token);
                m_values.push_back(v);
            }
        };
        struct TokenIntArray {
            btAlignedObjectArray<int>& m_values;
            TokenIntArray(btAlignedObjectArray<int>& intArray)
                : m_values(intArray)
            {
            }
            inline void add(const char* token)
            {
                float v = atoi(token);
                m_values.push_back(v);
            }
        };

        template <typename AddToken>
        void tokenize(const std::string& str, AddToken& tokenAdder, const std::string& delimiters = " \n")
        {
            std::string::size_type pos, lastPos = 0;
            while (true) {
                pos = str.find_first_of(delimiters, lastPos);
                if (pos == std::string::npos) {
                    pos = str.length();
                    if (pos != lastPos) {
                        tokenAdder.add(str.data() + lastPos);
                    }
                    break;
                }
                else {
                    if (pos != lastPos) {
                        tokenAdder.add(str.data() + lastPos);
                    }
                }
                lastPos = pos + 1;
            }
        }

        void readFloatArray(XMLElement* source, btAlignedObjectArray<float>& floatArray, int& componentStride)
        {
            int numVals, stride;
            XMLElement* array = source->FirstChildElement("float_array");
            if (array) {
                componentStride = 1;
                if (source->FirstChildElement("technique_common")->FirstChildElement("accessor")->QueryIntAttribute("stride", &stride) != XML_NO_ATTRIBUTE) {
                    componentStride = stride;
                }
                array->QueryIntAttribute("count", &numVals);
                TokenFloatArray adder(floatArray);
                floatArray.reserve(numVals);
                std::string txt = array->GetText();
                tokenize(array->GetText(), adder);
                assert(floatArray.size() == numVals);
            }
        }

        btVector3 getVector3FromXmlText(const char* text)
        {
            btVector3 vec(0, 0, 0);
            btAlignedObjectArray<float> floatArray;
            TokenFloatArray adder(floatArray);
            floatArray.reserve(3);
            tokenize(text, adder);
            assert(floatArray.size() == 3);
            if (floatArray.size() == 3) {
                vec.setValue(floatArray[0], floatArray[1], floatArray[2]);
            }
            return vec;
        }

        btVector4 getVector4FromXmlText(const char* text)
        {
            btVector4 vec(0, 0, 0, 0);
            btAlignedObjectArray<float> floatArray;
            TokenFloatArray adder(floatArray);
            floatArray.reserve(4);
            tokenize(text, adder);
            assert(floatArray.size() == 4);
            if (floatArray.size() == 4) {
                vec.setValue(floatArray[0], floatArray[1], floatArray[2], floatArray[3]);
            }
            return vec;
        }

        void readLibraryGeometries(XMLDocument& doc, btAlignedObjectArray<GLInstanceGraphicsShape>& visualShapes, btHashMap<btHashString, int>& name2Shape, float extraScaling)
        {
            btHashMap<btHashString, XMLElement*> allSources;
            btHashMap<btHashString, VertexSource> vertexSources;
            for (XMLElement* geometry = doc.RootElement()->FirstChildElement("library_geometries")->FirstChildElement("geometry");
                 geometry != NULL; geometry = geometry->NextSiblingElement("geometry")) {
                btAlignedObjectArray<btVector3> vertexPositions;
                btAlignedObjectArray<btVector3> vertexNormals;
                btAlignedObjectArray<int> indices;

                const char* geometryName = geometry->Attribute("id");
                for (XMLElement* mesh = geometry->FirstChildElement("mesh"); (mesh != NULL); mesh = mesh->NextSiblingElement("mesh")) {
                    XMLElement* vertices2 = mesh->FirstChildElement("vertices");

                    for (XMLElement* source = mesh->FirstChildElement("source"); source != NULL; source = source->NextSiblingElement("source")) {
                        const char* srcId = source->Attribute("id");
                        //				printf("source id=%s\n",srcId);
                        allSources.insert(srcId, source);
                    }
                    const char* vertexId = vertices2->Attribute("id");
                    //printf("vertices id=%s\n",vertexId);
                    VertexSource vs;
                    for (XMLElement* input = vertices2->FirstChildElement("input"); input != NULL; input = input->NextSiblingElement("input")) {
                        const char* sem = input->Attribute("semantic");
                        std::string semName(sem);
                        //					printf("sem=%s\n",sem);
                        //		const char* src = input->Attribute("source");
                        //					printf("src=%s\n",src);
                        const char* srcIdRef = input->Attribute("source");
                        std::string source_name;
                        source_name = std::string(srcIdRef);
                        source_name = source_name.erase(0, 1);
                        if (semName == "POSITION") {
                            vs.m_positionArrayId = source_name;
                        }
                        if (semName == "NORMAL") {
                            vs.m_normalArrayId = source_name;
                        }
                    }
                    vertexSources.insert(vertexId, vs);

                    btAlignedObjectArray<XMLElement*> trianglesAndPolylists;

                    for (XMLElement* primitive = mesh->FirstChildElement("triangles"); primitive; primitive = primitive->NextSiblingElement("triangles")) {
                        trianglesAndPolylists.push_back(primitive);
                    }
                    for (XMLElement* primitive = mesh->FirstChildElement("polylist"); primitive; primitive = primitive->NextSiblingElement("polylist")) {
                        trianglesAndPolylists.push_back(primitive);
                    }

                    for (int i = 0; i < trianglesAndPolylists.size(); i++) {
                        XMLElement* primitive = trianglesAndPolylists[i];
                        std::string positionSourceName;
                        std::string normalSourceName;
                        int primitiveCount;
                        primitive->QueryIntAttribute("count", &primitiveCount);
                        int indexStride = 1;
                        int posOffset = 0;
                        int normalOffset = 0;
                        int numIndices = 0;
                        {
                            for (XMLElement* input = primitive->FirstChildElement("input"); input != NULL; input = input->NextSiblingElement("input")) {
                                const char* sem = input->Attribute("semantic");
                                std::string semName(sem);
                                int offset = atoi(input->Attribute("offset"));
                                if ((offset + 1) > indexStride)
                                    indexStride = offset + 1;
                                //printf("sem=%s\n",sem);
                                //	const char* src = input->Attribute("source");

                                //printf("src=%s\n",src);
                                const char* srcIdRef = input->Attribute("source");
                                std::string source_name;
                                source_name = std::string(srcIdRef);
                                source_name = source_name.erase(0, 1);

                                if (semName == "VERTEX") {
                                    //now we have POSITION and possibly NORMAL too, using same index array (<p>)
                                    VertexSource* vs = vertexSources[source_name.c_str()];
                                    if (vs->m_positionArrayId.length()) {
                                        positionSourceName = vs->m_positionArrayId;
                                        posOffset = offset;
                                    }
                                    if (vs->m_normalArrayId.length()) {
                                        normalSourceName = vs->m_normalArrayId;
                                        normalOffset = offset;
                                    }
                                }
                                if (semName == "NORMAL") {
                                    btAssert(normalSourceName.length() == 0);
                                    normalSourceName = source_name;
                                    normalOffset = offset;
                                }
                            }
                            numIndices = primitiveCount * 3;
                        }
                        btAlignedObjectArray<float> positionFloatArray;
                        int posStride = 1;
                        XMLElement** sourcePtr = allSources[positionSourceName.c_str()];
                        if (sourcePtr) {
                            readFloatArray(*sourcePtr, positionFloatArray, posStride);
                        }
                        btAlignedObjectArray<float> normalFloatArray;
                        int normalStride = 1;
                        sourcePtr = allSources[normalSourceName.c_str()];
                        if (sourcePtr) {
                            readFloatArray(*sourcePtr, normalFloatArray, normalStride);
                        }
                        btAlignedObjectArray<int> curIndices;
                        curIndices.reserve(numIndices * indexStride);
                        TokenIntArray adder(curIndices);
                        std::string txt = primitive->FirstChildElement("p")->GetText();
                        tokenize(txt, adder);
                        assert(curIndices.size() == numIndices * indexStride);
                        int indexOffset = vertexPositions.size();

                        for (int index = 0; index < numIndices; index++) {
                            int posIndex = curIndices[index * indexStride + posOffset];
                            int normalIndex = curIndices[index * indexStride + normalOffset];
                            vertexPositions.push_back(btVector3(extraScaling * positionFloatArray[posIndex * 3 + 0],
                                extraScaling * positionFloatArray[posIndex * 3 + 1],
                                extraScaling * positionFloatArray[posIndex * 3 + 2]));

                            if (normalFloatArray.size() && (normalFloatArray.size() > normalIndex)) {
                                vertexNormals.push_back(btVector3(normalFloatArray[normalIndex * 3 + 0],
                                    normalFloatArray[normalIndex * 3 + 1],
                                    normalFloatArray[normalIndex * 3 + 2]));
                            }
                            else {
                                //add a dummy normal of length zero, so it is easy to detect that it is an invalid normal
                                vertexNormals.push_back(btVector3(0, 0, 0));
                            }
                        }
                        int curNumIndices = indices.size();
                        indices.resize(curNumIndices + numIndices);
                        for (int index = 0; index < numIndices; index++) {
                            indices[curNumIndices + index] = index + indexOffset;
                        }
                    } //if(primitive != NULL)
                } //for each mesh

                int shapeIndex = visualShapes.size();
                if (shapeIndex < MAX_VISUAL_SHAPES) {
                    GLInstanceGraphicsShape& visualShape = visualShapes.expand();
                    {
                        visualShape.m_vertices = new b3AlignedObjectArray<GLInstanceVertex>;
                        visualShape.m_indices = new b3AlignedObjectArray<int>;
                        int indexBase = 0;

                        btAssert(vertexNormals.size() == vertexPositions.size());
                        for (int v = 0; v < vertexPositions.size(); v++) {
                            GLInstanceVertex vtx;
                            vtx.xyzw[0] = vertexPositions[v].x();
                            vtx.xyzw[1] = vertexPositions[v].y();
                            vtx.xyzw[2] = vertexPositions[v].z();
                            vtx.xyzw[3] = 1.f;
                            vtx.normal[0] = vertexNormals[v].x();
                            vtx.normal[1] = vertexNormals[v].y();
                            vtx.normal[2] = vertexNormals[v].z();
                            vtx.uv[0] = 0.5f;
                            vtx.uv[1] = 0.5f;
                            visualShape.m_vertices->push_back(vtx);
                        }

                        for (int index = 0; index < indices.size(); index++) {
                            visualShape.m_indices->push_back(indices[index] + indexBase);
                        }

                        //b3Printf(" index_count =%dand vertexPositions.size=%d\n",indices.size(), vertexPositions.size());
                        indexBase = visualShape.m_vertices->size();
                        visualShape.m_numIndices = visualShape.m_indices->size();
                        visualShape.m_numvertices = visualShape.m_vertices->size();
                    }
                    //b3Printf("geometry name=%s\n",geometryName);
                    name2Shape.insert(geometryName, shapeIndex);
                }
                else {
                    b3Warning("DAE exceeds number of visual shapes (%d/%d)", shapeIndex, MAX_VISUAL_SHAPES);
                }

            } //for each geometry
        }

        void getUnitMeterScalingAndUpAxisTransform(XMLDocument& doc, btTransform& tr, float& unitMeterScaling, int clientUpAxis)
        {
            ///todo(erwincoumans) those up-axis transformations have been quickly coded without rigorous testing

            XMLElement* unitMeter = doc.RootElement()->FirstChildElement("asset")->FirstChildElement("unit");
            if (unitMeter) {
                const char* meterText = unitMeter->Attribute("meter");
                //printf("meterText=%s\n", meterText);
                unitMeterScaling = atof(meterText);
            }

            XMLElement* upAxisElem = doc.RootElement()->FirstChildElement("asset")->FirstChildElement("up_axis");
            if (upAxisElem) {
                switch (clientUpAxis) {
                case 1: {
                    std::string upAxisTxt = upAxisElem->GetText();
                    if (upAxisTxt == "X_UP") {
                        btQuaternion x2y(btVector3(0, 0, 1), SIMD_HALF_PI);
                        tr.setRotation(x2y);
                    }
                    if (upAxisTxt == "Y_UP") {
                        //assume Y_UP for now, to be compatible with assimp?
                        //client and COLLADA are both Z_UP so no transform needed (identity)
                    }
                    if (upAxisTxt == "Z_UP") {
                        btQuaternion z2y(btVector3(1, 0, 0), -SIMD_HALF_PI);
                        tr.setRotation(z2y);
                    }
                    break;
                }
                case 2: {
                    std::string upAxisTxt = upAxisElem->GetText();
                    if (upAxisTxt == "X_UP") {
                        btQuaternion x2z(btVector3(0, 1, 0), -SIMD_HALF_PI);
                        tr.setRotation(x2z);
                    }
                    if (upAxisTxt == "Y_UP") {
                        btQuaternion y2z(btVector3(1, 0, 0), SIMD_HALF_PI);
                        tr.setRotation(y2z);
                    }
                    if (upAxisTxt == "Z_UP") {
                        //client and COLLADA are both Z_UP so no transform needed (identity)
                    }
                    break;
                }
                case 0:
                default: {
                    //we don't support X or other up axis
                    btAssert(0);
                }
                };
            }
        }

        void readNodeHierarchy(XMLElement* node, btHashMap<btHashString, int>& name2Shape, btAlignedObjectArray<ColladaGraphicsInstance>& visualShapeInstances, const utils::btMatrix4x4& parentTransMat)
        {
            utils::btMatrix4x4 nodeTrans;
            nodeTrans.setIdentity();

            ///todo(erwincoumans) we probably have to read the elements 'translate', 'scale', 'rotate' and 'matrix' in-order and accumulate them...
            {
                for (XMLElement* transElem = node->FirstChildElement("matrix"); transElem; transElem = node->NextSiblingElement("matrix")) {
                    if (transElem->GetText()) {
                        btAlignedObjectArray<float> floatArray;
                        TokenFloatArray adder(floatArray);
                        tokenize(transElem->GetText(), adder);
                        if (floatArray.size() == 16) {
                            utils::btMatrix4x4 t(floatArray[0], floatArray[1], floatArray[2], floatArray[3],
                                floatArray[4], floatArray[5], floatArray[6], floatArray[7],
                                floatArray[8], floatArray[9], floatArray[10], floatArray[11],
                                floatArray[12], floatArray[13], floatArray[14], floatArray[15]);

                            nodeTrans = nodeTrans * t;
                        }
                        else {
                            b3Warning("Error: expected 16 elements in a <matrix> element, skipping\n");
                        }
                    }
                }
            }

            {
                for (XMLElement* transElem = node->FirstChildElement("translate"); transElem; transElem = node->NextSiblingElement("translate")) {
                    if (transElem->GetText()) {
                        btVector3 pos = getVector3FromXmlText(transElem->GetText());
                        //nodePos+= unitScaling*parentScaling*pos;
                        utils::btMatrix4x4 t;
                        t.setPureTranslation(pos);
                        nodeTrans = nodeTrans * t;
                    }
                }
            }
            {
                for (XMLElement* scaleElem = node->FirstChildElement("scale");
                     scaleElem != NULL; scaleElem = node->NextSiblingElement("scale")) {
                    if (scaleElem->GetText()) {
                        btVector3 scaling = getVector3FromXmlText(scaleElem->GetText());
                        utils::btMatrix4x4 t;
                        t.setPureScaling(scaling);
                        nodeTrans = nodeTrans * t;
                    }
                }
            }
            {
                for (XMLElement* rotateElem = node->FirstChildElement("rotate");
                     rotateElem != NULL; rotateElem = node->NextSiblingElement("rotate")) {
                    if (rotateElem->GetText()) {
                        //accumulate orientation
                        btVector4 rotate = getVector4FromXmlText(rotateElem->GetText());
                        btQuaternion orn(btVector3(rotate), btRadians(rotate[3])); //COLLADA DAE rotate is in degrees, convert to radians
                        utils::btMatrix4x4 t;
                        t.setPureRotation(orn);
                        nodeTrans = nodeTrans * t;
                    }
                }
            }

            nodeTrans = parentTransMat * nodeTrans;

            for (XMLElement* instanceGeom = node->FirstChildElement("instance_geometry");
                 instanceGeom != 0;
                 instanceGeom = instanceGeom->NextSiblingElement("instance_geometry")) {
                const char* geomUrl = instanceGeom->Attribute("url");
                //printf("node referring to geom %s\n", geomUrl);
                geomUrl++;
                int* shapeIndexPtr = name2Shape[geomUrl];
                if (shapeIndexPtr) {
                    //	int index = *shapeIndexPtr;
                    //printf("found geom with index %d\n", *shapeIndexPtr);
                    ColladaGraphicsInstance& instance = visualShapeInstances.expand();
                    instance.m_shapeIndex = *shapeIndexPtr;
                    instance.m_worldTransform = nodeTrans;
                }
                else {
                    b3Warning("geom not found\n");
                }
            }

            for (XMLElement* childNode = node->FirstChildElement("node");
                 childNode != NULL; childNode = childNode->NextSiblingElement("node")) {
                readNodeHierarchy(childNode, name2Shape, visualShapeInstances, nodeTrans);
            }
        }

        void readVisualSceneInstanceGeometries(XMLDocument& doc, btHashMap<btHashString, int>& name2Shape, btAlignedObjectArray<ColladaGraphicsInstance>& visualShapeInstances)
        {
            btHashMap<btHashString, XMLElement*> allVisualScenes;

            XMLElement* libVisualScenes = doc.RootElement()->FirstChildElement("library_visual_scenes");
            if (libVisualScenes == 0)
                return;

            {
                for (XMLElement* scene = libVisualScenes->FirstChildElement("visual_scene");
                     scene != NULL; scene = scene->NextSiblingElement("visual_scene")) {
                    const char* sceneName = scene->Attribute("id");
                    allVisualScenes.insert(sceneName, scene);
                }
            }

            XMLElement* scene = 0;
            {
                XMLElement* scenes = doc.RootElement()->FirstChildElement("scene");
                if (scenes) {
                    XMLElement* instanceSceneReference = scenes->FirstChildElement("instance_visual_scene");
                    if (instanceSceneReference) {
                        const char* instanceSceneUrl = instanceSceneReference->Attribute("url");
                        XMLElement** sceneInstancePtr = allVisualScenes[instanceSceneUrl + 1]; //skip #
                        if (sceneInstancePtr) {
                            scene = *sceneInstancePtr;
                        }
                    }
                }
            }

            if (scene) {
                for (XMLElement* node = scene->FirstChildElement("node");
                     node != NULL; node = node->NextSiblingElement("node")) {
                    utils::btMatrix4x4 identity;
                    identity.setIdentity();
                    btVector3 identScaling(1, 1, 1);
                    readNodeHierarchy(node, name2Shape, visualShapeInstances, identity);
                }
            }
        }

        void LoadMeshFromCollada(const char* relativeFileName, btAlignedObjectArray<GLInstanceGraphicsShape>& visualShapes, btAlignedObjectArray<ColladaGraphicsInstance>& visualShapeInstances, btTransform& upAxisTransform, float& unitMeterScaling, int clientUpAxis, interfaces::FileIOInterface* fileIO)
        {
            //	GLInstanceGraphicsShape* instance = 0;

            //usually COLLADA files don't have that many visual geometries/shapes
            visualShapes.reserve(MAX_VISUAL_SHAPES);

            float extraScaling = 1; //0.01;
            btHashMap<btHashString, int> name2ShapeIndex;

            char filename[1024];
            if (!fileIO->findResourcePath(relativeFileName, filename, 1024)) {
                b3Warning("File not found: %s\n", filename);
                return;
            }

            XMLDocument doc;
            //doc.Parse((const char*)filedata, 0, TIXML_ENCODING_UTF8);
            b3AlignedObjectArray<char> xmlString;
            int fileHandle = fileIO->fileOpen(filename, "r");
            if (fileHandle >= 0) {
                int size = fileIO->getFileSize(fileHandle);
                xmlString.resize(size);
                int actual = fileIO->fileRead(fileHandle, &xmlString[0], size);
                if (actual == size) {
                }
                fileIO->fileClose(fileHandle);
            }
            if (xmlString.size() == 0)
                return;

            if (doc.Parse(&xmlString[0], xmlString.size()) != XML_SUCCESS)
                //if (doc.LoadFile(filename) != XML_SUCCESS)
                return;

            //We need units to be in meter, so apply a scaling using the asset/units meter
            unitMeterScaling = 1;
            upAxisTransform.setIdentity();

            //Also we can optionally compensate all transforms using the asset/up_axis as well as unit meter scaling
            getUnitMeterScalingAndUpAxisTransform(doc, upAxisTransform, unitMeterScaling, clientUpAxis);

            utils::btMatrix4x4 ident;
            ident.setIdentity();

            readLibraryGeometries(doc, visualShapes, name2ShapeIndex, extraScaling);

            readVisualSceneInstanceGeometries(doc, name2ShapeIndex, visualShapeInstances);
        }
    } // namespace importers
} // namespace robot_bullet

#endif // ROBOT_BULLET_IMPORT_MESH