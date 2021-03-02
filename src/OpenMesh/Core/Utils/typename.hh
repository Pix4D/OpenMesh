#pragma once

/// Get an internal name for a type
/// Important, this is depends on compilers and versions, do NOT use in file formats!
/// This provides property type safety when only limited RTTI is available
/// Solution adapted from OpenVolumeMesh

#include <string>
#include <typeinfo>
#include <vector>
#include <OpenMesh/Core/Mesh/Handles.hh>
#include <OpenMesh/Core/Geometry/VectorT.hh>

namespace OpenMesh {

template <typename T>
std::string get_type_name()
{
#ifdef _MSC_VER
    // MSVC'S type_name returns only a friendly name with name() method,
    // to get a unique name use raw_name() method instead
    return typeid(T).raw_name();
#else
    // GCC and clang curently return mangled name as name(), there is no raw_name() method
    return typeid(T).name();
#endif
}

//----------------------get string for type recognition-- can be used in file format

inline std::string get_string_for_type(OpenMesh::FaceHandle){ return "facehandle";}
inline std::string get_string_for_type(OpenMesh::EdgeHandle){ return "edgehandle";}
inline std::string  get_string_for_type(OpenMesh::HalfedgeHandle){ return "halfedgehandle";}
inline std::string  get_string_for_type(OpenMesh::VertexHandle){ return "vertexhandle";}
inline std::string  get_string_for_type(OpenMesh::MeshHandle){ return "meshhandle";}

inline std::string  get_string_for_type(bool){ return "bool";}
inline std::string  get_string_for_type(char){ return "char";}
inline std::string  get_string_for_type(signed char){ return "signed char";}
inline std::string  get_string_for_type(double){ return "double";}
inline std::string  get_string_for_type(float){ return "float";}
inline std::string  get_string_for_type(int){ return "int";}
inline std::string  get_string_for_type(short){ return "short";}

inline std::string  get_string_for_type(unsigned char){ return "uchar";}
inline std::string  get_string_for_type(unsigned int){ return "uint";}
inline std::string  get_string_for_type(unsigned short){ return "ushort";}
inline std::string  get_string_for_type(unsigned long){ return "ulong";}

inline std::string  get_string_for_type(std::string){ return "std::string";}

template <typename T> std::string  get_string_for_type(T){return "unknown";}
template <typename T> std::string  get_string_for_type(std::vector<T>){ return "std::vector<" + get_string_for_type(T()) + ">";}

template <typename T, int Dim> std::string get_string_for_type(OpenMesh::VectorT<T, Dim>)
{ return "VectorT<" + get_string_for_type(T()) + ", " + std::to_string(Dim) + ">";}

}//namespace OpenMesh
