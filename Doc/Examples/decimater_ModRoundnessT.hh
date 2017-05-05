#include <OpenMesh/Tools/Decimater/DecimaterT.hh>
#include <OpenMesh/../../Doc/Examples/decimaterTestMesh.hh>

#include <OpenMesh/Tools/Decimater/ModRoundnessT.hh>

using namespace OpenMesh;

typedef Decimater::DecimaterT<ExampleMesh>          	ExampleDecimater;
typedef Decimater::ModRoundnessT<ExampleMesh>::Handle 	HModRoundness;

void decimateRoundness(ExampleMesh& mesh ){

    ExampleDecimater   decimater(mesh);  // a decimater object, connected to a mesh

    HModRoundness hModRoundness;						// use a roundness module
    decimater.add(hModRoundness);						// register the roundness module

    //in binary mode choose a minimal roundness OR angle
    decimater.module(hModRoundness).set_min_roundness(0.7, true);		// choose a minimal roundness value between 0 and 1.0 and set binary mode to true
    decimater.module(hModRoundness).set_min_angle(15.0, true);		// or choose a minimal angle in degrees. second argument gets ignored

    //in non-binary mode unset OR set minimal roundness
    decimater.module(hModRoundness).unset_min_roundness();			// unset minimal roundness
    decimater.module(hModRoundness).set_min_roundness(0.7, false);		// or choose a minimal roundness value between 0 and 1.0 and set binary mode to false

    decimater.initialize();
    decimater.decimate();
    // after decimation: remove decimated elements from the mesh
    mesh.garbage_collection();
}