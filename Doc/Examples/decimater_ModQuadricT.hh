#include <OpenMesh/Tools/Decimater/DecimaterT.hh>
#include <OpenMesh/../../Doc/Examples/decimaterTestMesh.hh>

#include <OpenMesh/Tools/Decimater/ModQuadricT.hh>

using namespace OpenMesh;

typedef Decimater::DecimaterT<ExampleMesh>          ExampleDecimater;
typedef Decimater::ModQuadricT<ExampleMesh>::Handle HModQuadric;

void decimateQuadric(ExampleMesh& mesh ){

    ExampleDecimater   decimater(mesh);  // a decimater object, connected to a mesh

    HModQuadric hModQuadric;      // use a quadric module
    decimater.add(hModQuadric); // register module at the decimater
    std::cout << decimater.module(hModQuadric).name() << std::endl; // module access

    /*
     * since we need exactly one priority module (non-binary)
     * we have to call set_binary(false) for our priority module
     * in the case of HModQuadric, unset_max_err() calls set_binary(false) internally
     */
    decimater.module(hModQuadric).unset_max_err();

    decimater.initialize();
    decimater.decimate();

    // after decimation: remove decimated elements from the mesh
    mesh.garbage_collection();
}
