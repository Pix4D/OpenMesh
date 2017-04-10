#pragma once
using namespace OpenMesh;

struct CustomTraitsVec3f : OpenMesh::DefaultTraits
{
    typedef OpenMesh::Vec3f Point;
};

typedef TriMesh_ArrayKernelT<CustomTraitsVec3f>         ExampleMesh;

