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
TEST_F(OpenMeshDocDecimater, DecimateAspectRatio) {
    {
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
      EXPECT_EQ(1,mesh_.n_faces()) << "Faces did not get decimated to face with lower aspect ratio";
    }

    {
      mesh_.clear();

      // Add some vertices
      TriMeshVec3f::VertexHandle vhandle[4];

      vhandle[0] = mesh_.add_vertex(TriMeshVec3f::Point(0.0, 0.0, 0.0));
      vhandle[1] = mesh_.add_vertex(TriMeshVec3f::Point(0.0, 1.0, 0.0));
      vhandle[2] = mesh_.add_vertex(TriMeshVec3f::Point(100.0, 100.0, 0.0));
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
      EXPECT_EQ(2,mesh_.n_faces()) << "Faces got decimated to face with higher aspect ratio";
    }

}

/*
 * Checking decimater_ModEdgeLengthT.hh
 */
TEST_F(OpenMeshDocDecimater, DecimateEdgeLength) {

    {
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


      decimateEdgeLength(mesh_);
      EXPECT_EQ(1,mesh_.n_faces()) << "Faces did not get decimated to face with shorter edges";
    }

    {
      mesh_.clear();

      // Add some vertices
      TriMeshVec3f::VertexHandle vhandle[4];

      vhandle[0] = mesh_.add_vertex(TriMeshVec3f::Point(0.0, 0.0, 0.0));
      vhandle[1] = mesh_.add_vertex(TriMeshVec3f::Point(0.0, 10.0, 0.0));
      vhandle[2] = mesh_.add_vertex(TriMeshVec3f::Point(10.0, 10.0, 0.0));
      vhandle[3] = mesh_.add_vertex(TriMeshVec3f::Point(10.0, 0.0, 0.0));

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


      decimateEdgeLength(mesh_);
      EXPECT_EQ(2,mesh_.n_faces()) << "An edge longer than 4 got decimated";
    }
}

/*
 * Checking decimater_ModHausdorffT.hh
 */
TEST_F(OpenMeshDocDecimater, DecimateHausdorff) {

    {
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


      decimateHausdorff(mesh_);
      EXPECT_EQ(1,mesh_.n_faces()) << "Faces did not get decimated but the Hausdorff tolerance is smaller 5.0";
    }

    {
      mesh_.clear();

      // Add some vertices
      TriMeshVec3f::VertexHandle vhandle[4];

      vhandle[0] = mesh_.add_vertex(TriMeshVec3f::Point(0.0, 0.0, 0.0));
      vhandle[1] = mesh_.add_vertex(TriMeshVec3f::Point(0.0, 100.0, 0.0));
      vhandle[2] = mesh_.add_vertex(TriMeshVec3f::Point(100.0, 100.0, 0.0));
      vhandle[3] = mesh_.add_vertex(TriMeshVec3f::Point(100.0, 0.0, 0.0));

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


      decimateHausdorff(mesh_);
      EXPECT_EQ(2,mesh_.n_faces()) << "Faces got decimated but the Hausdorff tolerance is larger 5.0";
    }
}


/*
 * Checking decimater_ModIndependentT.hh
 */
TEST_F(OpenMeshDocDecimater, DecimateIndependent) {

    {
      mesh_.clear();

      // Add some vertices
      TriMeshVec3f::VertexHandle vhandle[5];

      vhandle[0] = mesh_.add_vertex(TriMeshVec3f::Point(0.0, 0.0, 0.0));
      vhandle[1] = mesh_.add_vertex(TriMeshVec3f::Point(0.0, 1.0, 0.0));
      vhandle[2] = mesh_.add_vertex(TriMeshVec3f::Point(-1.0, -1.0, 0.0));
      vhandle[3] = mesh_.add_vertex(TriMeshVec3f::Point(1.0, -1.0, 0.0));
      vhandle[3] = mesh_.add_vertex(TriMeshVec3f::Point(1.0, -0.9, 0.0));

      // Add faces
      std::vector<TriMeshVec3f::VertexHandle> face_vhandles1;
      std::vector<TriMeshVec3f::VertexHandle> face_vhandles2;
      std::vector<TriMeshVec3f::VertexHandle> face_vhandles3;
      std::vector<TriMeshVec3f::VertexHandle> face_vhandles4;

      face_vhandles1.push_back(vhandle[0]);
      face_vhandles1.push_back(vhandle[1]);
      face_vhandles1.push_back(vhandle[3]);
      mesh_.add_face(face_vhandles1);

      face_vhandles2.push_back(vhandle[0]);
      face_vhandles2.push_back(vhandle[3]);
      face_vhandles2.push_back(vhandle[2]);
      mesh_.add_face(face_vhandles2);

      face_vhandles3.push_back(vhandle[0]);
      face_vhandles3.push_back(vhandle[2]);
      face_vhandles3.push_back(vhandle[1]);
      mesh_.add_face(face_vhandles3);

      face_vhandles4.push_back(vhandle[4]);
      face_vhandles4.push_back(vhandle[3]);
      face_vhandles4.push_back(vhandle[1]);
      mesh_.add_face(face_vhandles4);


      decimateIndependent(mesh_);
      EXPECT_EQ(3,mesh_.n_faces()) << "Faces dit not get decimated or dependent faces got decimated";
    }
}

/*
 * Checking decimater_ModNormalDeviationT.hh
 */
TEST_F(OpenMeshDocDecimater, DecimateNormalDeviation) {

    {
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


      decimateNormalDeviation(mesh_);
      EXPECT_EQ(1,mesh_.n_faces()) << "The normal would not deviate but the faces did not get decimated.";
    }

    {
      mesh_.clear();

      // Add some vertices
      TriMeshVec3f::VertexHandle vhandle[4];

      vhandle[0] = mesh_.add_vertex(TriMeshVec3f::Point(0.0, 0.0, 0.0));
      vhandle[1] = mesh_.add_vertex(TriMeshVec3f::Point(0.0, 1.0, 0.0));
      vhandle[2] = mesh_.add_vertex(TriMeshVec3f::Point(1.0, 1.0, 10.0));
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


      decimateNormalDeviation(mesh_);
      EXPECT_EQ(2,mesh_.n_faces()) << "Faces got decimated but the normal deviates";
    }
}

/*
 * Checking decimater_ModNormalFlippingT.hh
 */
TEST_F(OpenMeshDocDecimater, DecimateNormalFlipping) {

    {
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


      decimateNormalFlipping(mesh_);
      EXPECT_EQ(1,mesh_.n_faces()) << "The normal would not deviate but the faces did not get decimated.";
    }

    {
      mesh_.clear();

      // Add some vertices
      TriMeshVec3f::VertexHandle vhandle[4];

      vhandle[0] = mesh_.add_vertex(TriMeshVec3f::Point(0.0, 0.0, 10.0));
      vhandle[1] = mesh_.add_vertex(TriMeshVec3f::Point(0.0, 1.0, 0.0));
      vhandle[2] = mesh_.add_vertex(TriMeshVec3f::Point(1.0, 1.0, 10.0));
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


      decimateNormalFlipping(mesh_);
      EXPECT_EQ(2,mesh_.n_faces()) << "Faces got decimated but the normal deviates";
    }
}

/*
 * Checking decimater_ModQuadricT.hh
 */
TEST_F(OpenMeshDocDecimater, DecimateQuadratic) {

    {
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


      decimateQuadric(mesh_);
      EXPECT_EQ(1,mesh_.n_faces()) << "Faces did not get decimated to a face with small quadratic error.";
    }

    {
      mesh_.clear();

      // Add some vertices
      TriMeshVec3f::VertexHandle vhandle[4];

      vhandle[0] = mesh_.add_vertex(TriMeshVec3f::Point(0.0, 0.0, 10.0));
      vhandle[1] = mesh_.add_vertex(TriMeshVec3f::Point(0.0, 1.0, 0.0));
      vhandle[2] = mesh_.add_vertex(TriMeshVec3f::Point(10.0, 10.0, 10.0));
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


      decimateQuadric(mesh_);
      EXPECT_EQ(2,mesh_.n_faces()) << "Faces got decimated but the quadratic error was larger than 2";
    }
}


/*
 * Checking decimater_ModRoundnessT.hh
 */
TEST_F(OpenMeshDocDecimater, DecimateRoundness) {
    {
      mesh_.clear();

      // Add some vertices
      TriMeshVec3f::VertexHandle vhandle[4];

      vhandle[0] = mesh_.add_vertex(TriMeshVec3f::Point(0.0, 0.0, 0.0));
      vhandle[1] = mesh_.add_vertex(TriMeshVec3f::Point(0.0, 1.0, 0.0));
      vhandle[2] = mesh_.add_vertex(TriMeshVec3f::Point(2.0, 1.0, 0.0));
      vhandle[3] = mesh_.add_vertex(TriMeshVec3f::Point(1.0, 0.0, 0.0));

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


      decimateRoundness(mesh_);
      EXPECT_EQ(1,mesh_.n_faces()) << "Faces did not get decimated but the roundness would be greater 0.5.";
    }
    {
      mesh_.clear();

      // Add some vertices
      TriMeshVec3f::VertexHandle vhandle[4];

      vhandle[0] = mesh_.add_vertex(TriMeshVec3f::Point(0.0, 0.0, 0.0));
      vhandle[1] = mesh_.add_vertex(TriMeshVec3f::Point(0.0, 1.0, 0.0));
      vhandle[2] = mesh_.add_vertex(TriMeshVec3f::Point(10.0, 10.0, 100.0));
      vhandle[3] = mesh_.add_vertex(TriMeshVec3f::Point(10.0, 0.0, 0.0));

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


      decimateRoundness(mesh_);
      EXPECT_EQ(2,mesh_.n_faces()) << "Faces got decimated but roundness is to high";
    }
}


/*
 * Checking decimater.hh
 */
TEST_F(OpenMeshDocDecimater, Decimate) {
    {
      mesh_.clear();

      // Add some vertices
      TriMeshVec3f::VertexHandle vhandle[4];

      vhandle[0] = mesh_.add_vertex(TriMeshVec3f::Point(0.0, 0.0, 0.0));
      vhandle[1] = mesh_.add_vertex(TriMeshVec3f::Point(0.0, 1.0, 0.0));
      vhandle[2] = mesh_.add_vertex(TriMeshVec3f::Point(2.0, 1.0, 0.0));
      vhandle[3] = mesh_.add_vertex(TriMeshVec3f::Point(1.0, 0.0, 0.0));

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


      decimate(mesh_);
      EXPECT_EQ(1,mesh_.n_faces()) << "Faces did not get decimated";
    }
    {
      mesh_.clear();

      // Add some vertices
      TriMeshVec3f::VertexHandle vhandle[4];

      vhandle[0] = mesh_.add_vertex(TriMeshVec3f::Point(0.0, 0.0, 0.0));
      vhandle[1] = mesh_.add_vertex(TriMeshVec3f::Point(0.0, 1.0, 0.0));
      vhandle[2] = mesh_.add_vertex(TriMeshVec3f::Point(10.0, 10.0, 100.0));
      vhandle[3] = mesh_.add_vertex(TriMeshVec3f::Point(10.0, 0.0, 0.0));

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


      decimate(mesh_);
      EXPECT_EQ(2,mesh_.n_faces()) << "Faces got decimated illigally";
    }
}


  /*TriMeshVec3f::FaceHandle fh;
  for(const auto& f : mesh_.faces()){
      fh = f;
      break;
  }*/

/*  decimateAspectRatio(mesh_);

}
