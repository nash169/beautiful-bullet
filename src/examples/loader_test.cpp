#include <iostream>

#include <robot_bullet/utils/urdf/BulletLoader.hpp>

#include <robot_bullet/Simulator.hpp>

#include <robot_bullet/Agent.hpp>

using namespace robot_bullet;

int main(int argc, char** argv)
{
    Simulator sim;

    sim.addGround();

    Agent iiwa(sim, "models/iiwa/model.urdf");

    utils::urdf_parsing::BulletLoader loader;

    auto urdf_parsed = loader.parseMultiBody("models/iiwa/model.urdf", sim.world());

    // auto scene = utils::urdf_parsing::MeshShape::loadMesh("models/sphere.stl");

    // Assimp::Importer importer;

    // // And have it read the given file with some example postprocessing
    // // Usually - if speed is not the most important aspect for you - you'll
    // // probably to request more postprocessing than we do in this example.
    // const aiScene* scene = importer.ReadFile("models/sphere.stl",
    //     aiProcess_CalcTangentSpace | aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType);

    // std::cout << scene->mNumMeshes << std::endl;
    // std::cout << scene->mMeshes[0]->mNumFaces << std::endl;
    // std::cout << scene->mMeshes[0]->mFaces[0].mNumIndices << std::endl;

    return 0;
}