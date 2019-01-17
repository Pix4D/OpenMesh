
#ifdef ENABLE_EIGEN3_TEST

#include <gtest/gtest.h>
#include <Unittests/unittests_common.hh>
#include <iostream>

#include <OpenMesh/Core/Mesh/Traits.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

#include <OpenMesh/Core/Geometry/EigenVectorT.hh>

struct EigenTraits : OpenMesh::DefaultTraits {
    using Point = Eigen::Vector3d;
    using Normal = Eigen::Vector3d;

    using TexCoord2D = Eigen::Vector2d;
};

using EigenTriMesh = OpenMesh::TriMesh_ArrayKernelT<EigenTraits>;



template <class Scalar>
Eigen::Matrix<Scalar, 3, 1>
transformPoint(Eigen::Matrix<Scalar, 3, 4> const &trans,
               Eigen::Matrix<Scalar, 3, 1> const &point) {
    return trans.leftCols(3) * point + trans.col(3);
}

namespace {


class OpenMeshEigenTest : public testing::Test {

    protected:

        // This function is called before each test is run
        virtual void SetUp() {
            
            // Do some initial stuff with the member data here...
        }

        // This function is called after all tests are through
        virtual void TearDown() {


        }

        EigenTriMesh mesh_;
};

TEST_F(OpenMeshEigenTest, Test_external_norm) {


  EigenTriMesh::Normal normal(1,0,0);

  EigenTriMesh::Scalar result = norm(normal);

  EXPECT_EQ(result,1.0f ) << "Wrong norm";

  normal = EigenTriMesh::Normal(2,0,0);

  result = norm(normal);

  EXPECT_EQ(result,2.0f ) << "Wrong norm";
}

TEST_F(OpenMeshEigenTest, Test_external_sqrnorm) {


  EigenTriMesh::Normal normal(1,0,0);

  EigenTriMesh::Scalar result = sqrnorm(normal);

  EXPECT_EQ(result,1.0f ) << "Wrong norm";

  normal = EigenTriMesh::Normal(2,0,0);

  result = sqrnorm(normal);

  EXPECT_EQ(result,4.0f ) << "Wrong norm";
}

TEST_F(OpenMeshEigenTest, Test_external_normalize) {


  EigenTriMesh::Normal normal(2,0,0);

  EigenTriMesh::Normal normalized = normalize(normal);

  EXPECT_EQ(norm(normalized),1.0f ) << "Wrong norm after normalization";

  normal = EigenTriMesh::Normal(2,6,9);

  normalized = normalize(normal);

  EXPECT_EQ(norm(normalized),1.0f ) << "Wrong norm after normalization";

}

TEST_F(OpenMeshEigenTest, Test_external_cross_Product) {


  EigenTriMesh::Normal normal1(1,0,0);
  EigenTriMesh::Normal normal2(1,1,0);

  EigenTriMesh::Normal result = cross(normal1,normal2);

  EXPECT_EQ(result[0],0.0f ) << "Wrong result x direction";
  EXPECT_EQ(result[1],0.0f ) << "Wrong result y direction";
  EXPECT_EQ(result[2],1.0f ) << "Wrong result z direction";
}

TEST_F(OpenMeshEigenTest, Test_external_dot_Product) {


  EigenTriMesh::Normal normal1(1,0,0);
  EigenTriMesh::Normal normal2(1,1,0);
  EigenTriMesh::Normal normal3(1,1,1);
  EigenTriMesh::Normal normal4(2,4,6);

  EigenTriMesh::Scalar result  = dot(normal1,normal2);
  EigenTriMesh::Scalar result1 = dot(normal3,normal4);

  EXPECT_EQ(result,1.0f ) << "Wrong dot product";
  EXPECT_EQ(result1,12.0f ) << "Wrong dot product";

}


TEST_F(OpenMeshEigenTest, Test_Basic_setup) {

  // Add some vertices
  EigenTriMesh::VertexHandle vhandle[4];

  vhandle[0] = mesh_.add_vertex(EigenTriMesh::Point(0, 0, 0));
  vhandle[1] = mesh_.add_vertex(EigenTriMesh::Point(0, 1, 0));
  vhandle[2] = mesh_.add_vertex(EigenTriMesh::Point(1, 1, 0));
  vhandle[3] = mesh_.add_vertex(EigenTriMesh::Point(1, 0, 0));

  // Add two faces
  std::vector<EigenTriMesh::VertexHandle> face_vhandles;

  face_vhandles.push_back(vhandle[2]);
  face_vhandles.push_back(vhandle[1]);
  face_vhandles.push_back(vhandle[0]);

  mesh_.add_face(face_vhandles);

  face_vhandles.clear();

  face_vhandles.push_back(vhandle[2]);
  face_vhandles.push_back(vhandle[0]);
  face_vhandles.push_back(vhandle[3]);
  mesh_.add_face(face_vhandles);


  EXPECT_EQ(mesh_.n_faces(),2) << "Wrong number of faces";

}

TEST_F(OpenMeshEigenTest, test_normal_computation) {

  // Add some vertices
  EigenTriMesh::VertexHandle vhandle[4];

  mesh_.request_vertex_normals();
  mesh_.request_face_normals();

  vhandle[0] = mesh_.add_vertex(EigenTriMesh::Point(0, 0, 0));
  vhandle[1] = mesh_.add_vertex(EigenTriMesh::Point(0, 1, 0));
  vhandle[2] = mesh_.add_vertex(EigenTriMesh::Point(1, 1, 0));
  vhandle[3] = mesh_.add_vertex(EigenTriMesh::Point(1, 0, 0));

  // Add two faces
  std::vector<EigenTriMesh::VertexHandle> face_vhandles;

  face_vhandles.push_back(vhandle[2]);
  face_vhandles.push_back(vhandle[1]);
  face_vhandles.push_back(vhandle[0]);

  EigenTriMesh::FaceHandle face1 = mesh_.add_face(face_vhandles);

  face_vhandles.clear();

  face_vhandles.push_back(vhandle[2]);
  face_vhandles.push_back(vhandle[0]);
  face_vhandles.push_back(vhandle[3]);
  EigenTriMesh::FaceHandle face2 = mesh_.add_face(face_vhandles);

  mesh_.update_face_normals();


  EXPECT_EQ(mesh_.n_faces(),2) << "Wrong number of faces";

  EigenTriMesh::Normal normal = mesh_.normal(face1);

  EXPECT_EQ(normal[0],0.0f ) << "Wrong normal x direction";
  EXPECT_EQ(normal[1],0.0f ) << "Wrong normal y direction";
  EXPECT_EQ(normal[2],1.0f ) << "Wrong normal z direction";

  normal = mesh_.normal(face2);

  EXPECT_EQ(normal[0],0.0f ) << "Wrong normal x direction";
  EXPECT_EQ(normal[1],0.0f ) << "Wrong normal y direction";
  EXPECT_EQ(normal[2],1.0f ) << "Wrong normal z direction";

}

}

#endif
