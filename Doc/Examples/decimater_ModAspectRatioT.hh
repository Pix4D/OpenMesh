#include <OpenMesh/Tools/Decimater/DecimaterT.hh>
#include <OpenMesh/../../Doc/Examples/decimaterTestMesh.hh>

#include <OpenMesh/Tools/Decimater/ModAspectRatioT.hh>

using namespace OpenMesh;

typedef Decimater::DecimaterT<ExampleMesh>          	ExampleDecimater;
typedef Decimater::ModAspectRatioT<ExampleMesh>::Handle HModAspectRatio;

void decimateAspectRatio(ExampleMesh& mesh ){

        ExampleDecimater   decimater(mesh);  // a decimater object, connected to a mesh

	HModAspectRatio hModAspectRatio;					// use an aspect ratio module
	decimater.add(hModAspectRatio);						// register the aspect ratio module
	decimater.module(hModAspectRatio).set_binary(false); 			// if the aspect ratio module is not the primary module set binary to true;
	decimater.module(hModAspectRatio).set_aspect_ratio(3.0);   		// prevents collapsses which result in a smaller aspect ratio than the chosen minimum

	decimater.initialize();
	decimater.decimate();
	// after decimation: remove decimated elements from the mesh
	mesh.garbage_collection();
}
