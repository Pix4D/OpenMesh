#include <OpenMesh/Tools/Decimater/DecimaterT.hh>
#include <OpenMesh/../../Doc/Examples/decimaterTestMesh.hh>

#include <OpenMesh/Tools/Decimater/ModHausdorffT.hh>
#include <OpenMesh/Tools/Decimater/ModNormalDeviationT.hh>

using namespace OpenMesh;

typedef Decimater::DecimaterT<ExampleMesh>                      ExampleDecimater;
typedef Decimater::ModHausdorffT<ExampleMesh>::Handle 		HModHausdorff;
typedef Decimater::ModNormalDeviationT<ExampleMesh>::Handle 	HModNormalDeviation;

void decimateHausdorff(ExampleMesh& mesh ){

    ExampleDecimater   decimater(mesh);  // a decimater object, connected to a mesh

    HModHausdorff hModHausdorff;  						// use a hausdorff distance module
    decimater.add(hModHausdorff); 						// register the hausdorff distance module
    decimater.module(hModHausdorff).set_tolerance(10.0);			// set max Hausdorff distance tollerance

    //note that ModHausdorff only supports binary mode and has to be used in combination with a non-binary module
    HModNormalDeviation hModNormalDeviation;				// use a non-binary module primarily
    decimater.add(hModNormalDeviation); 				// register the module at the decimater
    decimater.module(hModNormalDeviation).set_binary(false); 		// choose non-binary mode

    decimater.initialize();
    decimater.decimate();
    // after decimation: remove decimated elements from the mesh
    mesh.garbage_collection();
}
