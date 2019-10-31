#include <gtest/gtest.h>
#include <Unittests/unittests_common.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>

#include <iostream>

namespace {

class OpenMeshPropertyManager : public OpenMeshBase {

    protected:

        // This function is called before each test is run
        virtual void SetUp() {
        }

        // This function is called after all tests are through
        virtual void TearDown() {

            // Do some final stuff with the member data here...
        }

    // Member already defined in OpenMeshBase
    //Mesh mesh_;  
};

/*
 * ====================================================================
 * General Tests
 * ====================================================================
 */

/*
 * Collapsing a tetrahedron
 */
TEST_F(OpenMeshPropertyManager, set_range_bool) {

  mesh_.clear();

  // Add some vertices
  Mesh::VertexHandle vhandle[4];

  vhandle[0] = mesh_.add_vertex(Mesh::Point(0, 0, 0));
  vhandle[1] = mesh_.add_vertex(Mesh::Point(0, 1, 0));
  vhandle[2] = mesh_.add_vertex(Mesh::Point(1, 1, 0));
  vhandle[3] = mesh_.add_vertex(Mesh::Point(0, 0, 1));

  // Add two faces
  std::vector<Mesh::VertexHandle> face_vhandles;

  face_vhandles.push_back(vhandle[0]);
  face_vhandles.push_back(vhandle[1]);
  face_vhandles.push_back(vhandle[2]);
  mesh_.add_face(face_vhandles);

  face_vhandles.clear();

  face_vhandles.push_back(vhandle[0]);
  face_vhandles.push_back(vhandle[2]);
  face_vhandles.push_back(vhandle[3]);
  mesh_.add_face(face_vhandles);

  {
      OpenMesh::PropertyManager<
          OpenMesh::VPropHandleT<bool>> pm_v_bool(mesh_, "pm_v_bool");
      pm_v_bool.set_range(mesh_.vertices_begin(), mesh_.vertices_end(), false);
      for (int i = 0; i < 4; ++i)
          ASSERT_FALSE(pm_v_bool[vhandle[i]]);
      pm_v_bool.set_range(mesh_.vertices_begin(), mesh_.vertices_end(), true);
      for (int i = 0; i < 4; ++i)
          ASSERT_TRUE(pm_v_bool[vhandle[i]]);

      OpenMesh::PropertyManager<
          OpenMesh::EPropHandleT<bool>> pm_e_bool(mesh_, "pm_e_bool");
      pm_e_bool.set_range(mesh_.edges_begin(), mesh_.edges_end(), false);
      for (Mesh::EdgeIter e_it = mesh_.edges_begin(), f_end = mesh_.edges_end();
              e_it != f_end; ++e_it)
          ASSERT_FALSE(pm_e_bool[*e_it]);
      pm_e_bool.set_range(mesh_.edges_begin(), mesh_.edges_end(), true);
      for (Mesh::EdgeIter e_it = mesh_.edges_begin(), f_end = mesh_.edges_end();
              e_it != f_end; ++e_it)
          ASSERT_TRUE(pm_e_bool[*e_it]);

      OpenMesh::PropertyManager<
          OpenMesh::FPropHandleT<bool>> pm_f_bool(mesh_, "pm_f_bool");
      pm_f_bool.set_range(mesh_.faces_begin(), mesh_.faces_end(), false);
      for (Mesh::FaceIter f_it = mesh_.faces_begin(), f_end = mesh_.faces_end();
              f_it != f_end; ++f_it)
          ASSERT_FALSE(pm_f_bool[*f_it]);
      pm_f_bool.set_range(mesh_.faces_begin(), mesh_.faces_end(), true);
      for (Mesh::FaceIter f_it = mesh_.faces_begin(), f_end = mesh_.faces_end();
              f_it != f_end; ++f_it)
          ASSERT_TRUE(pm_f_bool[*f_it]);
  }

  /*
   * Same thing again, this time with C++11 ranges.
   */
  {
      OpenMesh::PropertyManager<
          OpenMesh::VPropHandleT<bool>> pm_v_bool(mesh_, "pm_v_bool2");
      pm_v_bool.set_range(mesh_.vertices(), false);
      for (int i = 0; i < 4; ++i)
          ASSERT_FALSE(pm_v_bool[vhandle[i]]);
      pm_v_bool.set_range(mesh_.vertices(), true);
      for (int i = 0; i < 4; ++i)
          ASSERT_TRUE(pm_v_bool[vhandle[i]]);

      OpenMesh::PropertyManager<
          OpenMesh::EPropHandleT<bool>> pm_e_bool(mesh_, "pm_e_bool2");
      pm_e_bool.set_range(mesh_.edges(), false);
      for (Mesh::EdgeIter e_it = mesh_.edges_begin(), f_end = mesh_.edges_end();
              e_it != f_end; ++e_it)
          ASSERT_FALSE(pm_e_bool[*e_it]);
      pm_e_bool.set_range(mesh_.edges(), true);
      for (Mesh::EdgeIter e_it = mesh_.edges_begin(), f_end = mesh_.edges_end();
              e_it != f_end; ++e_it)
          ASSERT_TRUE(pm_e_bool[*e_it]);

      OpenMesh::PropertyManager<
          OpenMesh::FPropHandleT<bool>> pm_f_bool(mesh_, "pm_f_bool2");
      pm_f_bool.set_range(mesh_.faces(), false);
      for (Mesh::FaceIter f_it = mesh_.faces_begin(), f_end = mesh_.faces_end();
              f_it != f_end; ++f_it)
          ASSERT_FALSE(pm_f_bool[*f_it]);
      pm_f_bool.set_range(mesh_.faces(), true);
      for (Mesh::FaceIter f_it = mesh_.faces_begin(), f_end = mesh_.faces_end();
              f_it != f_end; ++f_it)
          ASSERT_TRUE(pm_f_bool[*f_it]);
  }
}

/*
 * In sequence:
 * - add a persistent property to a mesh
 * - retrieve an existing property of a mesh and modify it
 * - obtain a non-owning property handle
 * - attempt to obtain a non-owning handle to a non-existing property (throws)
 */
TEST_F(OpenMeshPropertyManager, cpp11_persistent_and_non_owning_properties) {
    auto vh = mesh_.add_vertex({0,0,0}); // Dummy vertex to attach properties to

    const auto prop_name = "pm_v_test_property";

    ASSERT_FALSE((OpenMesh::hasProperty<OpenMesh::VertexHandle, int>(mesh_, prop_name)));

    {
        auto prop = OpenMesh::getOrMakeProperty<OpenMesh::VertexHandle, int>(mesh_, prop_name);
        prop[vh] = 100;
        // End of scope, property persists
    }

    ASSERT_TRUE((OpenMesh::hasProperty<OpenMesh::VertexHandle, int>(mesh_, prop_name)));

    {
        // Since a property of the same name and type already exists, this refers to the existing property.
        auto prop = OpenMesh::getOrMakeProperty<OpenMesh::VertexHandle, int>(mesh_, prop_name);
        ASSERT_EQ(100, prop[vh]);
        prop[vh] = 200;
        // End of scope, property persists
    }

    ASSERT_TRUE((OpenMesh::hasProperty<OpenMesh::VertexHandle, int>(mesh_, prop_name)));

    {
        // Acquire non-owning handle to the property, knowing it exists
        auto prop = OpenMesh::getProperty<OpenMesh::VertexHandle, int>(mesh_, prop_name);
        ASSERT_EQ(200, prop[vh]);
    }

    ASSERT_TRUE((OpenMesh::hasProperty<OpenMesh::VertexHandle, int>(mesh_, prop_name)));

    {
        // Attempt to acquire non-owning handle for a non-existing property
        auto code_that_throws = [&](){
            OpenMesh::getProperty<OpenMesh::VertexHandle, int>(mesh_, "wrong_prop_name");
        };
        ASSERT_THROW(code_that_throws(), std::runtime_error);
    }

    ASSERT_TRUE((OpenMesh::hasProperty<OpenMesh::VertexHandle, int>(mesh_, prop_name)));
}


TEST_F(OpenMeshPropertyManager, property_copy_construction) {
  for (int i = 0; i < 1000000; ++i)
    mesh_.add_vertex(Mesh::Point());

  // unnamed
  {
    auto prop1 = OpenMesh::VProp<int>(mesh_);
    for (auto vh : mesh_.vertices())
      prop1[vh] = vh.idx()*2-13;

    auto  prop2 = prop1; // prop1 and prop2 should be two different properties with the same content

    prop1.set_range(mesh_.vertices(), 0);

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], -13);
  }

  // named
  {
    auto prop1 = OpenMesh::VProp<int>(mesh_, "ids");
    for (auto vh : mesh_.vertices())
      prop1[vh] = vh.idx()*2-13;

    auto prop2 = prop1; // prop1 and prop2 should refere to the same property

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], -13);

    prop1.set_range(mesh_.vertices(), 0);

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 0);
  }
}

TEST_F(OpenMeshPropertyManager, property_move_construction) {
  for (int i = 0; i < 1000000; ++i)
    mesh_.add_vertex(Mesh::Point());

  // unnamed
  {
    auto prop1 = OpenMesh::VProp<int>(mesh_);
    for (auto vh : mesh_.vertices())
      prop1[vh] = vh.idx()*2-13;

    auto t_start = std::chrono::high_resolution_clock::now();
    auto prop2 = std::move(prop1);
    auto t_end = std::chrono::high_resolution_clock::now();
    std::cout << "move constructing property from temporary took " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end-t_start).count() << "ms" << std::endl;

    EXPECT_FALSE(prop1.isValid()) << "prop1 should have been invalidated";

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], -13);
  }

  // named
  {
    auto prop1 = OpenMesh::VProp<int>(mesh_, "ids");
    for (auto vh : mesh_.vertices())
      prop1[vh] = vh.idx()*2-13;

    auto t_start = std::chrono::high_resolution_clock::now();
    auto prop2 = std::move(prop1); // prop1 and prop2 should refere to the same property
    auto t_end = std::chrono::high_resolution_clock::now();
    std::cout << "move constructing from named took " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end-t_start).count() << "ms" << std::endl;

    EXPECT_TRUE(prop1.isValid()) << "named properties cannot be invalidated";

    EXPECT_EQ(prop1[OpenMesh::VertexHandle(0)], -13) << "property is not valid anymore";
    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], -13) << "did not copy property correctly";

    prop1.set_range(mesh_.vertices(), 0);

    EXPECT_EQ(prop1[OpenMesh::VertexHandle(0)], 0);
    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 0);
  }
}


TEST_F(OpenMeshPropertyManager, property_copying_same_mesh) {

  for (int i = 0; i < 1000000; ++i)
    mesh_.add_vertex(Mesh::Point());

  // unnamed to unnamed
  {
    auto prop1 = OpenMesh::VProp<int>(mesh_, 3);
    auto prop2 = OpenMesh::VProp<int>(mesh_, 0);
    EXPECT_EQ(prop1[OpenMesh::VertexHandle(0)], 3) << "Property not initialized correctly";
    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 0) << "Property not initialized correctly";

    for (auto vh : mesh_.vertices())
      prop1[vh] = vh.idx()*2-13;

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 0) << "Property not initialized correctly";

    auto t_start = std::chrono::high_resolution_clock::now();
    prop2 = prop1;
    auto t_end = std::chrono::high_resolution_clock::now();
    std::cout << "copying property temporary to temporary took " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end-t_start).count() << "ms" << std::endl;

    EXPECT_EQ(prop1[OpenMesh::VertexHandle(0)], -13) << "Temporary property got destroyed";

    prop1.set_range(mesh_.vertices(), 0);

    EXPECT_EQ(prop1[OpenMesh::VertexHandle(0)], 0) << "Property not copied correctly";
    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";
  }

  // unnamed to named
  {
    auto prop1 = OpenMesh::VProp<int>(mesh_);
    auto prop2 = OpenMesh::VProp<int>(mesh_, "ids", 0);
    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 0) << "Property not initialized correctly";

    for (auto vh : mesh_.vertices())
      prop1[vh] = vh.idx()*2-13;

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 0) << "Property not initialized correctly";

    auto t_start = std::chrono::high_resolution_clock::now();
    prop2 = prop1;
    auto t_end = std::chrono::high_resolution_clock::now();
    std::cout << "copying property temporary to named took  " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end-t_start).count() << "ms" << std::endl;

    EXPECT_EQ(prop1[OpenMesh::VertexHandle(0)], -13) << "Temporary property got destroyed";

    prop1.set_range(mesh_.vertices(), 0);

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";

    auto prop3 = OpenMesh::VProp<int>(mesh_, "ids");
    EXPECT_EQ(prop3[OpenMesh::VertexHandle(0)], -13) << "property with name 'ids' was not correctly changed";
  }

  // named to unnamed
  {
    auto prop1 = OpenMesh::VProp<int>(mesh_, "ids2");
    auto prop2 = OpenMesh::VProp<int>(mesh_);
    prop2.set_range(mesh_.vertices(), 0);
    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 0) << "Property not initialized correctly";

    for (auto vh : mesh_.vertices())
      prop1[vh] = vh.idx()*2-13;

    auto t_start = std::chrono::high_resolution_clock::now();
    prop2 = prop1;
    auto t_end = std::chrono::high_resolution_clock::now();
    std::cout << "copying property named to temporary took  " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end-t_start).count() << "ms" << std::endl;

    prop1.set_range(mesh_.vertices(), 0);

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";

  }

  // named to named (different names)
  {
    auto prop1 = OpenMesh::VProp<int>(mesh_, "ids3");
    auto prop2 = OpenMesh::VProp<int>(mesh_, "ids4");
    prop2.set_range(mesh_.vertices(), 0);
    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 0) << "Property not initialized correctly";

    for (auto vh : mesh_.vertices())
      prop1[vh] = vh.idx()*2-13;

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 0) << "Property not initialized correctly";

    auto t_start = std::chrono::high_resolution_clock::now();
    prop2 = prop1;
    auto t_end = std::chrono::high_resolution_clock::now();
    std::cout << "copying property named to named with different name took  " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end-t_start).count() << "ms" << std::endl;

    prop1.set_range(mesh_.vertices(), 0);

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";
  }

  // named to named (same names)
  {
    auto prop1 = OpenMesh::VProp<int>(mesh_, "ids5");
    auto prop2 = OpenMesh::VProp<int>(mesh_, "ids5");

    for (auto vh : mesh_.vertices())
      prop1[vh] = vh.idx()*2-13;

    EXPECT_EQ(prop1[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";
    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";

    auto t_start = std::chrono::high_resolution_clock::now();
    prop2 = prop1; // this should be a no op
    auto t_end = std::chrono::high_resolution_clock::now();
    std::cout << "copying property named to named with same name took " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end-t_start).count() << "ms" << std::endl;

    EXPECT_EQ(prop1[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";
    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";

    prop1.set_range(mesh_.vertices(), 42);

    EXPECT_EQ(prop1[OpenMesh::VertexHandle(0)], 42) << "Property not copied correctly";
    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 42) << "Property not copied correctly";

    auto prop3 = OpenMesh::VProp<int>(mesh_, "ids5");
    EXPECT_EQ(prop3[OpenMesh::VertexHandle(0)], 42) << "Property not copied correctly";
  }
}


TEST_F(OpenMeshPropertyManager, property_moving_same_mesh) {

  for (int i = 0; i < 1000000; ++i)
    mesh_.add_vertex(Mesh::Point());

  // unnamed to unnamed
  {
    auto prop1 = OpenMesh::VProp<int>(mesh_);
    auto prop2 = OpenMesh::VProp<int>(mesh_);
    prop2.set_range(mesh_.vertices(), 0);

    for (auto vh : mesh_.vertices())
      prop1[vh] = vh.idx()*2-13;

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 0) << "Property not initialized correctly";

    auto t_start = std::chrono::high_resolution_clock::now();
    prop2 = std::move(prop1); // this should be cheap
    auto t_end = std::chrono::high_resolution_clock::now();
    std::cout << "moving property temporary to temporary took  " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end-t_start).count() << "ms" << std::endl;

    EXPECT_FALSE(prop1.isValid()) << "prop1 not invalidated after moving";

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";
  }

  // unnamed to named
  {
    auto prop1 = OpenMesh::VProp<int>(mesh_);
    auto prop2 = OpenMesh::VProp<int>(mesh_, "ids");
    prop2.set_range(mesh_.vertices(), 0);
    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 0) << "Property not initialized correctly";

    for (auto vh : mesh_.vertices())
      prop1[vh] = vh.idx()*2-13;

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 0) << "Property not initialized correctly";

    auto t_start = std::chrono::high_resolution_clock::now();
    prop2 = std::move(prop1);
    auto t_end = std::chrono::high_resolution_clock::now();
    std::cout << "moving property temporary to named took  " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end-t_start).count() << "ms" << std::endl;

    EXPECT_FALSE(prop1.isValid()) << "prop1 not invalidated after moving";

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";

    auto prop3 = OpenMesh::VProp<int>(mesh_, "ids");
    EXPECT_EQ(prop3[OpenMesh::VertexHandle(0)], -13) << "property with name 'ids' was not correctly changed";
  }

  // named to unnamed
  {
    auto prop1 = OpenMesh::VProp<int>(mesh_, "ids2");
    auto prop2 = OpenMesh::VProp<int>(mesh_);
    prop2.set_range(mesh_.vertices(), 0);
    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 0) << "Property not initialized correctly";

    for (auto vh : mesh_.vertices())
      prop1[vh] = vh.idx()*2-13;

    auto t_start = std::chrono::high_resolution_clock::now();
    prop2 = std::move(prop1); // moving named properties will not invalidate the property and will copy the data
    auto t_end = std::chrono::high_resolution_clock::now();
    std::cout << "moving property named to temporary took  " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end-t_start).count() << "ms" << std::endl;

    EXPECT_TRUE(prop1.isValid()) << "named prop1 should not be invalidated by moving";

    prop1.set_range(mesh_.vertices(), 0);

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";

  }

  // named to named (different names)
  {
    auto prop1 = OpenMesh::VProp<int>(mesh_, "ids3");
    auto prop2 = OpenMesh::VProp<int>(mesh_, "ids4");
    prop2.set_range(mesh_.vertices(), 0);
    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 0) << "Property not initialized correctly";

    for (auto vh : mesh_.vertices())
      prop1[vh] = vh.idx()*2-13;

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 0) << "Property not initialized correctly";

    auto t_start = std::chrono::high_resolution_clock::now();
    prop2 = std::move(prop1); // moving named properties will not invalidate the property and will copy the data
    auto t_end = std::chrono::high_resolution_clock::now();
    std::cout << "moving property named to named with different name took  " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end-t_start).count() << "ms" << std::endl;

    EXPECT_TRUE(prop1.isValid()) << "named prop1 should not be invalidated by moving";

    prop1.set_range(mesh_.vertices(), 0);

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";
  }

  // named to named (same names)
  {
    auto prop1 = OpenMesh::VProp<int>(mesh_, "ids5");
    auto prop2 = OpenMesh::VProp<int>(mesh_, "ids5");

    for (auto vh : mesh_.vertices())
      prop1[vh] = vh.idx()*2-13;

    EXPECT_EQ(prop1[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";
    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";

    auto t_start = std::chrono::high_resolution_clock::now();
    prop2 = std::move(prop1); // this should be a no op
    auto t_end = std::chrono::high_resolution_clock::now();
    std::cout << "moving property named to named with same name took  " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end-t_start).count() << "ms" << std::endl;

    EXPECT_TRUE(prop1.isValid()) << "named prop1 should not be invalidated by moving";

    EXPECT_EQ(prop1[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";
    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";

    prop1.set_range(mesh_.vertices(), 0);

    EXPECT_EQ(prop1[OpenMesh::VertexHandle(0)], 0) << "Property not copied correctly";
    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 0) << "Property not copied correctly";

    auto prop3 = OpenMesh::VProp<int>(mesh_, "ids5");
    EXPECT_EQ(prop3[OpenMesh::VertexHandle(0)], 0) << "Property not copied correctly";
  }
}



TEST_F(OpenMeshPropertyManager, property_copying_different_mesh) {

  for (int i = 0; i < 1000000; ++i)
    mesh_.add_vertex(Mesh::Point());

  auto copy = mesh_;
  for (int i = 0; i < 10; ++i)
    copy.add_vertex(Mesh::Point());

  // unnamed to unnamed
  {
    auto prop1 = OpenMesh::VProp<int>(mesh_, 3);
    auto prop2 = OpenMesh::VProp<int>(copy, 0);
    EXPECT_EQ(prop1[OpenMesh::VertexHandle(0)], 3) << "Property not initialized correctly";
    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 0) << "Property not initialized correctly";

    for (auto vh : mesh_.vertices())
      prop1[vh] = vh.idx()*2-13;

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 0) << "Property not initialized correctly";

    auto t_start = std::chrono::high_resolution_clock::now();
    prop2 = prop1;
    auto t_end = std::chrono::high_resolution_clock::now();
    std::cout << "copying property temporary to temporary took " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end-t_start).count() << "ms" << std::endl;

    EXPECT_EQ(prop1[OpenMesh::VertexHandle(0)], -13) << "Temporary property got destroyed";

    prop1.set_range(mesh_.vertices(), 0);

    EXPECT_EQ(prop1[OpenMesh::VertexHandle(0)], 0) << "Property not copied correctly";
    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";
    EXPECT_NO_FATAL_FAILURE(prop2[OpenMesh::VertexHandle(static_cast<int>(copy.n_vertices())-1)]) << "Property not correctly resized";
  }

  // unnamed to named
  {
    auto prop1 = OpenMesh::VProp<int>(mesh_);
    auto prop2 = OpenMesh::VProp<int>(copy, "ids", 0);
    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 0) << "Property not initialized correctly";

    for (auto vh : mesh_.vertices())
      prop1[vh] = vh.idx()*2-13;

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 0) << "Property not initialized correctly";

    auto t_start = std::chrono::high_resolution_clock::now();
    prop2 = prop1;
    auto t_end = std::chrono::high_resolution_clock::now();
    std::cout << "copying property temporary to named took  " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end-t_start).count() << "ms" << std::endl;

    EXPECT_EQ(prop1[OpenMesh::VertexHandle(0)], -13) << "Temporary property got destroyed";

    prop1.set_range(mesh_.vertices(), 0);

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";

    auto prop3 = OpenMesh::VProp<int>(copy, "ids");
    EXPECT_EQ(prop3[OpenMesh::VertexHandle(0)], -13) << "property with name 'ids' was not correctly changed";
  }

  // named to unnamed
  {
    auto prop1 = OpenMesh::VProp<int>(mesh_, "ids2");
    auto prop2 = OpenMesh::VProp<int>(copy);
    prop2.set_range(mesh_.vertices(), 0);
    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 0) << "Property not initialized correctly";

    for (auto vh : mesh_.vertices())
      prop1[vh] = vh.idx()*2-13;

    auto t_start = std::chrono::high_resolution_clock::now();
    prop2 = prop1;
    auto t_end = std::chrono::high_resolution_clock::now();
    std::cout << "copying property named to temporary took  " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end-t_start).count() << "ms" << std::endl;

    prop1.set_range(mesh_.vertices(), 0);

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";

  }

  // named to named (different names)
  {
    auto prop1 = OpenMesh::VProp<int>(mesh_, "ids3");
    auto prop2 = OpenMesh::VProp<int>(copy, "ids4");
    prop2.set_range(mesh_.vertices(), 0);
    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 0) << "Property not initialized correctly";

    for (auto vh : mesh_.vertices())
      prop1[vh] = vh.idx()*2-13;

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 0) << "Property not initialized correctly";

    auto t_start = std::chrono::high_resolution_clock::now();
    prop2 = prop1;
    auto t_end = std::chrono::high_resolution_clock::now();
    std::cout << "copying property named to named with different name took  " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end-t_start).count() << "ms" << std::endl;

    prop1.set_range(mesh_.vertices(), 0);

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";
  }

  // named to named (same names)
  {
    auto prop1 = OpenMesh::VProp<int>(mesh_, "ids5");
    auto prop2 = OpenMesh::VProp<int>(copy, "ids5");

    for (auto vh : mesh_.vertices())
      prop1[vh] = vh.idx()*2-13;

    EXPECT_EQ(prop1[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";

    auto t_start = std::chrono::high_resolution_clock::now();
    prop2 = prop1; // this should be a no op
    auto t_end = std::chrono::high_resolution_clock::now();
    std::cout << "copying property named to named with same name took " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end-t_start).count() << "ms" << std::endl;

    EXPECT_EQ(prop1[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";
    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";

    prop1.set_range(mesh_.vertices(), 42);

    EXPECT_EQ(prop1[OpenMesh::VertexHandle(0)], 42) << "Property not copied correctly";
    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";

    auto prop3 = OpenMesh::VProp<int>(mesh_, "ids5");
    EXPECT_EQ(prop3[OpenMesh::VertexHandle(0)], 42) << "Property not copied correctly";
    auto prop4 = OpenMesh::VProp<int>(copy, "ids5");
    EXPECT_EQ(prop4[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";
  }
}


TEST_F(OpenMeshPropertyManager, property_moving_different_mesh) {

  for (int i = 0; i < 1000000; ++i)
    mesh_.add_vertex(Mesh::Point());

  auto copy = mesh_;
  for (int i = 0; i < 10; ++i)
    copy.add_vertex(Mesh::Point());

  // unnamed to unnamed
  {
    auto prop1 = OpenMesh::VProp<int>(mesh_);
    auto prop2 = OpenMesh::VProp<int>(copy);
    prop2.set_range(mesh_.vertices(), 0);

    for (auto vh : mesh_.vertices())
      prop1[vh] = vh.idx()*2-13;

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 0) << "Property not initialized correctly";

    auto t_start = std::chrono::high_resolution_clock::now();
    prop2 = std::move(prop1); // this should be cheap
    auto t_end = std::chrono::high_resolution_clock::now();
    std::cout << "moving property temporary to temporary took  " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end-t_start).count() << "ms" << std::endl;

    EXPECT_FALSE(prop1.isValid()) << "prop1 not invalidated after moving";

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";
    EXPECT_NO_FATAL_FAILURE(prop2[OpenMesh::VertexHandle(static_cast<int>(copy.n_vertices())-1)]) << "Property not correctly resized";
  }

  // unnamed to named
  {
    auto prop1 = OpenMesh::VProp<int>(mesh_);
    auto prop2 = OpenMesh::VProp<int>(copy, "ids");
    prop2.set_range(mesh_.vertices(), 0);
    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 0) << "Property not initialized correctly";

    for (auto vh : mesh_.vertices())
      prop1[vh] = vh.idx()*2-13;

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 0) << "Property not initialized correctly";

    auto t_start = std::chrono::high_resolution_clock::now();
    prop2 = std::move(prop1);
    auto t_end = std::chrono::high_resolution_clock::now();
    std::cout << "moving property temporary to named took  " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end-t_start).count() << "ms" << std::endl;

    EXPECT_FALSE(prop1.isValid()) << "prop1 not invalidated after moving";

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";

    auto prop3 = OpenMesh::VProp<int>(copy, "ids");
    EXPECT_EQ(prop3[OpenMesh::VertexHandle(0)], -13) << "property with name 'ids' was not correctly changed";
  }

  // named to unnamed
  {
    auto prop1 = OpenMesh::VProp<int>(mesh_, "ids2");
    auto prop2 = OpenMesh::VProp<int>(copy);
    prop2.set_range(mesh_.vertices(), 0);
    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 0) << "Property not initialized correctly";

    for (auto vh : mesh_.vertices())
      prop1[vh] = vh.idx()*2-13;

    auto t_start = std::chrono::high_resolution_clock::now();
    prop2 = std::move(prop1); // moving named properties will not invalidate the property and will copy the data
    auto t_end = std::chrono::high_resolution_clock::now();
    std::cout << "moving property named to temporary took  " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end-t_start).count() << "ms" << std::endl;

    EXPECT_TRUE(prop1.isValid()) << "named prop1 should not be invalidated by moving";

    prop1.set_range(mesh_.vertices(), 0);

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";

  }

  // named to named (different names)
  {
    auto prop1 = OpenMesh::VProp<int>(mesh_, "ids3");
    auto prop2 = OpenMesh::VProp<int>(copy, "ids4");
    prop2.set_range(mesh_.vertices(), 0);
    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 0) << "Property not initialized correctly";

    for (auto vh : mesh_.vertices())
      prop1[vh] = vh.idx()*2-13;

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], 0) << "Property not initialized correctly";

    auto t_start = std::chrono::high_resolution_clock::now();
    prop2 = std::move(prop1); // moving named properties will not invalidate the property and will copy the data
    auto t_end = std::chrono::high_resolution_clock::now();
    std::cout << "moving property named to named with different name took  " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end-t_start).count() << "ms" << std::endl;

    EXPECT_TRUE(prop1.isValid()) << "named prop1 should not be invalidated by moving";

    prop1.set_range(mesh_.vertices(), 0);

    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";
  }

  // named to named (same names)
  {
    auto prop1 = OpenMesh::VProp<int>(mesh_, "ids5");
    auto prop2 = OpenMesh::VProp<int>(copy, "ids5");

    for (auto vh : mesh_.vertices())
      prop1[vh] = vh.idx()*2-13;

    EXPECT_EQ(prop1[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";

    auto t_start = std::chrono::high_resolution_clock::now();
    prop2 = std::move(prop1); // should copy
    auto t_end = std::chrono::high_resolution_clock::now();
    std::cout << "moving property named to named with same name took  " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end-t_start).count() << "ms" << std::endl;

    EXPECT_TRUE(prop1.isValid()) << "named prop1 should not be invalidated by moving";

    EXPECT_EQ(prop1[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";
    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";

    prop1.set_range(mesh_.vertices(), 42);

    EXPECT_EQ(prop1[OpenMesh::VertexHandle(0)], 42) << "Property not copied correctly";
    EXPECT_EQ(prop2[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";

    auto prop3 = OpenMesh::VProp<int>(mesh_, "ids5");
    EXPECT_EQ(prop3[OpenMesh::VertexHandle(0)], 42) << "Property not copied correctly";
    auto prop4 = OpenMesh::VProp<int>(copy, "ids5");
    EXPECT_EQ(prop4[OpenMesh::VertexHandle(0)], -13) << "Property not copied correctly";
  }
}



}
