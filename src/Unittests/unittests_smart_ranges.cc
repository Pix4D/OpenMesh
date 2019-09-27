#include <gtest/gtest.h>
#include <Unittests/unittests_common.hh>

#include <OpenMesh/Core/Mesh/SmartHandles.hh>

#include <iostream>
#include <chrono>

namespace {

class OpenMeshSmartRanges : public OpenMeshBase {

protected:

  // This function is called before each test is run
  virtual void SetUp() {

    mesh_.clear();

    // Add some vertices
    Mesh::VertexHandle vhandle[8];
    vhandle[0] = mesh_.add_vertex(Mesh::Point(-1, -1,  1));
    vhandle[1] = mesh_.add_vertex(Mesh::Point( 1, -1,  1));
    vhandle[2] = mesh_.add_vertex(Mesh::Point( 1,  1,  1));
    vhandle[3] = mesh_.add_vertex(Mesh::Point(-1,  1,  1));
    vhandle[4] = mesh_.add_vertex(Mesh::Point(-1, -1, -1));
    vhandle[5] = mesh_.add_vertex(Mesh::Point( 1, -1, -1));
    vhandle[6] = mesh_.add_vertex(Mesh::Point( 1,  1, -1));
    vhandle[7] = mesh_.add_vertex(Mesh::Point(-1,  1, -1));

    // Add six faces to form a cube
    std::vector<Mesh::VertexHandle> face_vhandles;

    face_vhandles.clear();
    face_vhandles.push_back(vhandle[0]);
    face_vhandles.push_back(vhandle[1]);
    face_vhandles.push_back(vhandle[3]);
    mesh_.add_face(face_vhandles);

    face_vhandles.clear();
    face_vhandles.push_back(vhandle[1]);
    face_vhandles.push_back(vhandle[2]);
    face_vhandles.push_back(vhandle[3]);
    mesh_.add_face(face_vhandles);

    //=======================

    face_vhandles.clear();
    face_vhandles.push_back(vhandle[7]);
    face_vhandles.push_back(vhandle[6]);
    face_vhandles.push_back(vhandle[5]);
    mesh_.add_face(face_vhandles);

    face_vhandles.clear();
    face_vhandles.push_back(vhandle[7]);
    face_vhandles.push_back(vhandle[5]);
    face_vhandles.push_back(vhandle[4]);
    mesh_.add_face(face_vhandles);

    //=======================

    face_vhandles.clear();
    face_vhandles.push_back(vhandle[1]);
    face_vhandles.push_back(vhandle[0]);
    face_vhandles.push_back(vhandle[4]);
    mesh_.add_face(face_vhandles);

    face_vhandles.clear();
    face_vhandles.push_back(vhandle[1]);
    face_vhandles.push_back(vhandle[4]);
    face_vhandles.push_back(vhandle[5]);
    mesh_.add_face(face_vhandles);

    //=======================

    face_vhandles.clear();
    face_vhandles.push_back(vhandle[2]);
    face_vhandles.push_back(vhandle[1]);
    face_vhandles.push_back(vhandle[5]);
    mesh_.add_face(face_vhandles);

    face_vhandles.clear();
    face_vhandles.push_back(vhandle[2]);
    face_vhandles.push_back(vhandle[5]);
    face_vhandles.push_back(vhandle[6]);
    mesh_.add_face(face_vhandles);


    //=======================

    face_vhandles.clear();
    face_vhandles.push_back(vhandle[3]);
    face_vhandles.push_back(vhandle[2]);
    face_vhandles.push_back(vhandle[6]);
    mesh_.add_face(face_vhandles);

    face_vhandles.clear();
    face_vhandles.push_back(vhandle[3]);
    face_vhandles.push_back(vhandle[6]);
    face_vhandles.push_back(vhandle[7]);
    mesh_.add_face(face_vhandles);

    //=======================

    face_vhandles.clear();
    face_vhandles.push_back(vhandle[0]);
    face_vhandles.push_back(vhandle[3]);
    face_vhandles.push_back(vhandle[7]);
    mesh_.add_face(face_vhandles);

    face_vhandles.clear();
    face_vhandles.push_back(vhandle[0]);
    face_vhandles.push_back(vhandle[7]);
    face_vhandles.push_back(vhandle[4]);
    mesh_.add_face(face_vhandles);


    // Test setup:
    //
    //
    //    3 ======== 2
    //   /          /|
    //  /          / |      z
    // 0 ======== 1  |      |
    // |          |  |      |   y
    // |  7       |  6      |  /
    // |          | /       | /
    // |          |/        |/
    // 4 ======== 5         -------> x
    //

    // Check setup
    EXPECT_EQ(18u, mesh_.n_edges() )     << "Wrong number of Edges";
    EXPECT_EQ(36u, mesh_.n_halfedges() ) << "Wrong number of HalfEdges";
    EXPECT_EQ(8u, mesh_.n_vertices() )   << "Wrong number of vertices";
    EXPECT_EQ(12u, mesh_.n_faces() )     << "Wrong number of faces";
  }

  // This function is called after all tests are through
  virtual void TearDown() {

    // Do some final stuff with the member data here...

    mesh_.clear();
  }

  // Member already defined in OpenMeshBase
  //Mesh mesh_;
};

/*
 * ====================================================================
 * Define tests below
 * ====================================================================
 */


template <typename HandleT>
struct F
{
  int operator()(HandleT ) { return 1; }
};

/* Test if smart ranges work
 */
TEST_F(OpenMeshSmartRanges, Sum)
{
  auto one = [](OpenMesh::VertexHandle ) { return 1; };
  EXPECT_EQ(mesh_.vertices().sum(one), mesh_.n_vertices());
  EXPECT_EQ(mesh_.vertices().sum(F<OpenMesh::VertexHandle>()), mesh_.n_vertices());
  EXPECT_EQ(mesh_.halfedges().sum(F<OpenMesh::HalfedgeHandle>()), mesh_.n_halfedges());
  EXPECT_EQ(mesh_.edges().sum(F<OpenMesh::EdgeHandle>()), mesh_.n_edges());
  EXPECT_EQ(mesh_.faces().sum(F<OpenMesh::FaceHandle>()), mesh_.n_faces());

  for (auto vh : mesh_.vertices())
    EXPECT_EQ(vh.vertices().sum(F<OpenMesh::VertexHandle>()), mesh_.valence(vh));
  for (auto vh : mesh_.vertices())
    EXPECT_EQ(vh.faces().sum(F<OpenMesh::FaceHandle>()), mesh_.valence(vh));
  for (auto vh : mesh_.vertices())
    EXPECT_EQ(vh.outgoing_halfedges().sum(F<OpenMesh::HalfedgeHandle>()), mesh_.valence(vh));
  for (auto vh : mesh_.vertices())
    EXPECT_EQ(vh.incoming_halfedges().sum(F<OpenMesh::HalfedgeHandle>()), mesh_.valence(vh));

  for (auto fh : mesh_.faces())
    EXPECT_EQ(fh.vertices().sum(F<OpenMesh::VertexHandle>()), mesh_.valence(fh));
  for (auto fh : mesh_.faces())
    EXPECT_EQ(fh.halfedges().sum(F<OpenMesh::HalfedgeHandle>()), mesh_.valence(fh));
  for (auto fh : mesh_.faces())
    EXPECT_EQ(fh.edges().sum(F<OpenMesh::EdgeHandle>()), mesh_.valence(fh));
  for (auto fh : mesh_.faces())
    EXPECT_EQ(fh.faces().sum(F<OpenMesh::FaceHandle>()), 3);
}



}
