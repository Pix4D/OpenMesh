#include <gtest/gtest.h>
#include <Unittests/unittests_common.hh>
#include <iostream>

#include "OpenMesh/../../Doc/Examples/decimater.hh"
#include "OpenMesh/../../Doc/Examples/decimater_ModAspectRatioT.hh"
#include "OpenMesh/../../Doc/Examples/decimater_ModEdgeLengthT.hh"
#include "OpenMesh/../../Doc/Examples/decimater_ModHausdorffT.hh"
#include "OpenMesh/../../Doc/Examples/decimater_ModIndependentT.hh"
#include "OpenMesh/../../Doc/Examples/decimater_ModNormalDeviationT.hh"
#include "OpenMesh/../../Doc/Examples/decimater_ModNormalFlippingT.hh"
#include "OpenMesh/../../Doc/Examples/decimater_ModQuadricT.hh"
#include "OpenMesh/../../Doc/Examples/decimater_ModRoundnessT.hh"


typedef OpenMesh::TriMesh_ArrayKernelT<CustomTraitsVec3f> TriMeshVec3f;


class OpenMeshDocDecimater : public testing::Test {

    protected:

        // This function is called before each test is run
        virtual void SetUp() {
        }

        // This function is called after all tests are through
        virtual void TearDown() {

            // Do some final stuff with the member data here...
        }

    // This member will be accessible in all tests
   TriMeshVec3f mesh_;

};



namespace {

/*
 * ====================================================================
 * Define tests below
 * ====================================================================
 */

/*
 * Checking decimater_ModAspectRatioT.hh
 */
TEST_F(OpenMeshDocDecimater, Instance_Doc_Decimater) {

  mesh_.clear();

  // Add some vertices
  TriMeshVec3f::VertexHandle vhandle[4];

  vhandle[0] = mesh_.add_vertex(TriMeshVec3f::Point(0.0, 0.0, 0.0));
  vhandle[1] = mesh_.add_vertex(TriMeshVec3f::Point(0.0, 1.0, 0.0));
  vhandle[2] = mesh_.add_vertex(TriMeshVec3f::Point(1.0, 1.0, 0.0));
  vhandle[3] = mesh_.add_vertex(TriMeshVec3f::Point(0.5, 0.5, 0.0));

  // Add faces
  std::vector<TriMeshVec3f::VertexHandle> face_vhandles1;
  std::vector<TriMeshVec3f::VertexHandle> face_vhandles2;

  face_vhandles1.push_back(vhandle[0]);
  face_vhandles1.push_back(vhandle[1]);
  face_vhandles1.push_back(vhandle[3]);
  mesh_.add_face(face_vhandles1);

  face_vhandles2.push_back(vhandle[3]);
  face_vhandles2.push_back(vhandle[1]);
  face_vhandles2.push_back(vhandle[2]);
  mesh_.add_face(face_vhandles2);

  decimateAspectRatio(mesh_);

  //TODO check if faces collapsed


  //TODO:
  decimate(mesh_);
  decimateAspectRatio(mesh_);
  decimateEdgeLength(mesh_);
  decimateHausdorff(mesh_);
  decimateIndependent(mesh_);
  decimateNormalDeviation(mesh_);
  decimateNormalFlipping(mesh_);
  decimateQuadric(mesh_);
  decimateRoundness(mesh_);

}


}
