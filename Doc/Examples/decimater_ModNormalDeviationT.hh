#include <OpenMesh/Tools/Decimater/DecimaterT.hh>
#include <OpenMesh/../../Doc/Examples/decimaterTestMesh.hh>

#include <OpenMesh/Tools/Decimater/ModNormalDeviationT.hh>

using namespace OpenMesh;

typedef Decimater::DecimaterT<ExampleMesh>                      ExampleDecimater;
typedef Decimater::ModNormalDeviationT<ExampleMesh>::Handle 	HModNormalDeviation;

void decimateNormalDeviation(ExampleMesh& mesh ){

    ExampleDecimater   decimater(mesh);  // a decimater object, connected to a mesh

    HModNormalDeviation hModNormalDeviation;				// use a normal deviation module
    decimater.add(hModNormalDeviation); 				// register deviation module at the decimater
    decimater.module(hModNormalDeviation).set_binary(false); 		// if the normal deviation module is not the primary module set binary to true;
    decimater.module(hModNormalDeviation).set_normal_deviation(15.0);	// set max angle between normals in degrees

    decimater.initialize();
    decimater.decimate();
    // after decimation: remove decimated elements from the mesh
    mesh.garbage_collection();
}
