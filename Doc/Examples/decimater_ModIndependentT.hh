#include <OpenMesh/Tools/Decimater/DecimaterT.hh>
#include <OpenMesh/../../Doc/Examples/decimaterTestMesh.hh>

#include <OpenMesh/Tools/Decimater/ModNormalDeviationT.hh>
#include <OpenMesh/Tools/Decimater/ModIndependentSetsT.hh>

using namespace OpenMesh;

typedef Decimater::DecimaterT<ExampleMesh>                      ExampleDecimater;
typedef Decimater::ModIndependentSetsT<ExampleMesh>::Handle 	HModIndependent;
typedef Decimater::ModNormalDeviationT<ExampleMesh>::Handle 	HModNormalDeviation;

void decimateIndependent(ExampleMesh& mesh ){

    ExampleDecimater   decimater(mesh);  // a decimater object, connected to a mesh

    HModIndependent hModIndependent;  				// use a independence module
    decimater.add(hModIndependent);                             // register the module at the decimater

    //note that ModIndependent only supports binary mode and has to be used in combination with a non-binary module
    HModNormalDeviation hModNormalDeviation;			// use a non-binary module primarily
    decimater.add(hModNormalDeviation); 			// register the module at the decimater
    decimater.module(hModNormalDeviation).set_binary(false); 	// choose non-binary mode

    decimater.initialize();
    decimater.decimate();
    // after decimation: remove decimated elements from the mesh
    mesh.garbage_collection();
}
