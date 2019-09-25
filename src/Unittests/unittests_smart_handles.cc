#include <gtest/gtest.h>
#include <Unittests/unittests_common.hh>

#include <OpenMesh/Core/Mesh/SmartHandles.hh>

#include <iostream>
#include <chrono>

namespace {

class OpenMeshSmartHandles : public OpenMeshBase {

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



/* Test if navigation operations on smart handles yield the expected element
 */
TEST_F(OpenMeshSmartHandles, SimpleNavigation)
{
  for (auto vh : mesh_.vertices())
  {
    auto svh = OpenMesh::make_smart(vh, mesh_);
    EXPECT_EQ(mesh_.halfedge_handle(vh), svh.halfedge()) << "outgoing halfedge of vertex does not match";
  }

  for (auto heh : mesh_.halfedges())
  {
    auto sheh = OpenMesh::make_smart(heh, mesh_);
    EXPECT_EQ(mesh_.next_halfedge_handle(heh),     sheh.next()) << "next halfedge of halfedge does not match";
    EXPECT_EQ(mesh_.prev_halfedge_handle(heh),     sheh.prev()) << "prevt halfedge of halfedge does not match";
    EXPECT_EQ(mesh_.opposite_halfedge_handle(heh), sheh.opp())  << "opposite halfedge of halfedge does not match";
    EXPECT_EQ(mesh_.to_vertex_handle(heh),         sheh.to())   << "to vertex handle of halfedge does not match";
    EXPECT_EQ(mesh_.from_vertex_handle(heh),       sheh.from()) << "from vertex handle of halfedge does not match";
    EXPECT_EQ(mesh_.face_handle(heh),              sheh.face()) << "face handle of halfedge does not match";
  }

  for (auto eh : mesh_.edges())
  {
    auto seh = OpenMesh::make_smart(eh, mesh_);
    EXPECT_EQ(mesh_.halfedge_handle(eh, 0), seh.h0()) << "halfedge 0 of edge does not match";
    EXPECT_EQ(mesh_.halfedge_handle(eh, 1), seh.h1()) << "halfedge 1 of edge does not match";
    EXPECT_EQ(mesh_.from_vertex_handle(mesh_.halfedge_handle(eh, 0)), seh.v0()) << "first vertex of edge does not match";
    EXPECT_EQ(mesh_.to_vertex_handle  (mesh_.halfedge_handle(eh, 0)), seh.v1()) << "second vertex of edge does not match";
  }

  for (auto fh : mesh_.faces())
  {
    auto sfh = OpenMesh::make_smart(fh, mesh_);
    EXPECT_EQ(mesh_.halfedge_handle(fh), sfh.halfedge()) << "halfedge of face does not match";
  }
}


/* Test if ranges yield the same elements when using smart handles
 */
TEST_F(OpenMeshSmartHandles, SimpleRanges)
{
  for (auto vh : mesh_.vertices())
  {
    auto svh = OpenMesh::make_smart(vh, mesh_);
    {
      std::vector<OpenMesh::VertexHandle> handles0;
      std::vector<OpenMesh::VertexHandle> handles1;
      for (auto h : mesh_.vv_range(vh))
        handles0.push_back(h);
      for (auto h : svh.vertices())
        handles1.push_back(h);
      EXPECT_EQ(handles0, handles1) << "vertex range of vertex does not match";
    }
    {
      std::vector<OpenMesh::HalfedgeHandle> handles0;
      std::vector<OpenMesh::HalfedgeHandle> handles1;
      for (auto h : mesh_.voh_range(vh))
        handles0.push_back(h);
      for (auto h : svh.outgoing_halfedges())
        handles1.push_back(h);
      EXPECT_EQ(handles0, handles1) << "outgoing halfedge range of vertex does not match";
    }
    {
      std::vector<OpenMesh::HalfedgeHandle> handles0;
      std::vector<OpenMesh::HalfedgeHandle> handles1;
      for (auto h : mesh_.vih_range(vh))
        handles0.push_back(h);
      for (auto h : svh.incoming_halfedges())
        handles1.push_back(h);
      EXPECT_EQ(handles0, handles1) << "incoming halfedge range of vertex does not match";
    }
    {
      std::vector<OpenMesh::EdgeHandle> handles0;
      std::vector<OpenMesh::EdgeHandle> handles1;
      for (auto h : mesh_.ve_range(vh))
        handles0.push_back(h);
      for (auto h : svh.edges())
        handles1.push_back(h);
      EXPECT_EQ(handles0, handles1) << "edge range of vertex does not match";
    }
    {
      std::vector<OpenMesh::FaceHandle> handles0;
      std::vector<OpenMesh::FaceHandle> handles1;
      for (auto h : mesh_.vf_range(vh))
        handles0.push_back(h);
      for (auto h : svh.faces())
        handles1.push_back(h);
      EXPECT_EQ(handles0, handles1) << "face range of vertex does not match";
    }
  }

  for (auto fh : mesh_.faces())
  {
    auto sfh = OpenMesh::make_smart(fh, mesh_);
    {
      std::vector<OpenMesh::VertexHandle> handles0;
      std::vector<OpenMesh::VertexHandle> handles1;
      for (auto h : mesh_.fv_range(fh))
        handles0.push_back(h);
      for (auto h : sfh.vertices())
        handles1.push_back(h);
      EXPECT_EQ(handles0, handles1) << "vertex range of face does not match";
    }
    {
      std::vector<OpenMesh::HalfedgeHandle> handles0;
      std::vector<OpenMesh::HalfedgeHandle> handles1;
      for (auto h : mesh_.fh_range(fh))
        handles0.push_back(h);
      for (auto h : sfh.halfedges())
        handles1.push_back(h);
      EXPECT_EQ(handles0, handles1) << "halfedge range of face does not match";
    }
    {
      std::vector<OpenMesh::EdgeHandle> handles0;
      std::vector<OpenMesh::EdgeHandle> handles1;
      for (auto h : mesh_.fe_range(fh))
        handles0.push_back(h);
      for (auto h : sfh.edges())
        handles1.push_back(h);
      EXPECT_EQ(handles0, handles1) << "edge range of face does not match";
    }
    {
      std::vector<OpenMesh::FaceHandle> handles0;
      std::vector<OpenMesh::FaceHandle> handles1;
      for (auto h : mesh_.ff_range(fh))
        handles0.push_back(h);
      for (auto h : sfh.faces())
        handles1.push_back(h);
      EXPECT_EQ(handles0, handles1) << "face range of face does not match";
    }
  }
}


/* Test a chain of navigation on a cube
 */
TEST_F(OpenMeshSmartHandles, ComplicatedNavigtaion)
{
  for (auto vh : mesh_.vertices())
  {
    auto svh = OpenMesh::make_smart(vh, mesh_);
    EXPECT_EQ(mesh_.next_halfedge_handle(
              mesh_.opposite_halfedge_handle(
              mesh_.halfedge_handle(vh))),
              svh.out().opp().next());
    EXPECT_EQ(mesh_.prev_halfedge_handle(
              mesh_.prev_halfedge_handle(
              mesh_.opposite_halfedge_handle(
              mesh_.next_halfedge_handle(
              mesh_.next_halfedge_handle(
              mesh_.halfedge_handle(vh)))))),
              svh.out().next().next().opp().prev().prev());
    EXPECT_EQ(mesh_.face_handle(
              mesh_.opposite_halfedge_handle(
              mesh_.halfedge_handle(
              mesh_.face_handle(
              mesh_.opposite_halfedge_handle(
              mesh_.next_halfedge_handle(
              mesh_.halfedge_handle(vh))))))),
              svh.out().next().opp().face().halfedge().opp().face());
  }
}


/* Test performance of smart handles
 */
TEST_F(OpenMeshSmartHandles, Performance)
{
  int n_tests = 10000000;

  auto t_before_old = std::chrono::high_resolution_clock::now();

  std::vector<OpenMesh::HalfedgeHandle> halfedges0;
  for (int i = 0; i < n_tests; ++i)
  {
    for (auto vh : mesh_.vertices())
    {
      auto heh = mesh_.prev_halfedge_handle(
                 mesh_.prev_halfedge_handle(
                 mesh_.opposite_halfedge_handle(
                 mesh_.next_halfedge_handle(
                 mesh_.next_halfedge_handle(
                 mesh_.halfedge_handle(vh))))));
      if (i == 0)
        halfedges0.push_back(heh);
    }
  }

  auto t_after_old = std::chrono::high_resolution_clock::now();

  std::vector<OpenMesh::HalfedgeHandle> halfedges1;
  for (int i = 0; i < n_tests; ++i)
  {
    for (auto vh : mesh_.vertices())
    {
      auto svh = OpenMesh::make_smart(vh, mesh_);
      auto heh = svh.out().next().next().opp().prev().prev();
      if (i == 0)
        halfedges1.push_back(heh);
    }
  }

  auto t_after_new = std::chrono::high_resolution_clock::now();

  std::cout << "Conventional navigation took " << std::chrono::duration_cast<std::chrono::milliseconds>(t_after_old-t_before_old).count() << "ms" << std::endl;
  std::cout << "SmartHandle  navigation took " << std::chrono::duration_cast<std::chrono::milliseconds>(t_after_new-t_after_old ).count() << "ms" << std::endl;

  EXPECT_EQ(halfedges0, halfedges1) << "halfedges do not match";

}

}
