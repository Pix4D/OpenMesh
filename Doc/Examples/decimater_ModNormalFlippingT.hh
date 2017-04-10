#include <OpenMesh/Tools/Decimater/DecimaterT.hh>
#include <OpenMesh/../../Doc/Examples/decimaterTestMesh.hh>

#include <OpenMesh/Tools/Decimater/ModNormalFlippingT.hh>
#include <OpenMesh/Tools/Decimater/ModNormalDeviationT.hh>

using namespace OpenMesh;

typedef Decimater::DecimaterT<ExampleMesh>                      ExampleDecimater;
typedef Decimater::ModNormalFlippingT<ExampleMesh>::Handle 	HModNormalFlipping;
typedef Decimater::ModNormalDeviationT<ExampleMesh>::Handle 	HModNormalDeviation;

void decimateNormalFlipping(ExampleMesh& mesh ){

    ExampleDecimater   decimater(mesh);  // a decimater object, connected to a mesh

    HModNormalFlipping hModNormalFlipping;  				// use a normal flipping module
    decimater.add(hModNormalFlipping); 					// register the normal flipping module
    decimater.module(hModNormalFlipping).set_max_normal_deviation(45.0);	// set the maximum normal deviation after collapse

    //note that ModNormalFlipping only supports binary mode and has to be used in combination with a non-binary module
    HModNormalDeviation hModNormalDeviation;				// use a non-binary module primarily
    decimater.add(hModNormalDeviation); 					// register the module at the decimater
    decimater.module(hModNormalDeviation).set_binary(false); 		// choose non-binary mode

    decimater.initialize();
    decimater.decimate();
    // after decimation: remove decimated elements from the mesh
    mesh.garbage_collection();
}
