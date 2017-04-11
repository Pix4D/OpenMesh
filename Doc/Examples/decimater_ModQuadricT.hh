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

    /*
     * since we need exactly one priority module (non-binary)
     * we have to call set_binary(false) for our priority module
     * in the case of HModQuadric, unset_max_err() calls set_binary(false) internally
     * or sprcify binary mode calling set_max_err()
     */
    decimater.module(hModQuadric).set_max_err(2.0, false); // sets maximum quadratic error to 2 and binary mode to false
    decimater.initialize();
    decimater.decimate();

    // after decimation: remove decimated elements from the mesh
    mesh.garbage_collection();
}
