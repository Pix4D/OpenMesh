/* ========================================================================= *
 *                                                                           *
 *                               OpenMesh                                    *
 *           Copyright (c) 2001-2015, RWTH-Aachen University                 *
 *           Department of Computer Graphics and Multimedia                  *
 *                          All rights reserved.                             *
 *                            www.openmesh.org                               *
 *                                                                           *
 *---------------------------------------------------------------------------*
 * This file is part of OpenMesh.                                            *
 *---------------------------------------------------------------------------*
 *                                                                           *
 * Redistribution and use in source and binary forms, with or without        *
 * modification, are permitted provided that the following conditions        *
 * are met:                                                                  *
 *                                                                           *
 * 1. Redistributions of source code must retain the above copyright notice, *
 *    this list of conditions and the following disclaimer.                  *
 *                                                                           *
 * 2. Redistributions in binary form must reproduce the above copyright      *
 *    notice, this list of conditions and the following disclaimer in the    *
 *    documentation and/or other materials provided with the distribution.   *
 *                                                                           *
 * 3. Neither the name of the copyright holder nor the names of its          *
 *    contributors may be used to endorse or promote products derived from   *
 *    this software without specific prior written permission.               *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED *
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A           *
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER *
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,  *
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,       *
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR        *
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF    *
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING      *
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS        *
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.              *
 *                                                                           *
 * ========================================================================= */

/*===========================================================================*\
 *                                                                           *
 *   $Revision$                                                         *
 *   $Date$                   *
 *                                                                           *
\*===========================================================================*/


#ifndef OPENMESH_IO_OM2FORMAT_HH
#define OPENMESH_IO_OM2FORMAT_HH


//=== INCLUDES ================================================================

#include <OpenMesh/Core/System/config.h>
#include <OpenMesh/Core/IO/SR_store.hh>
#include <OpenMesh/Core/IO/Options.hh>
#include <OpenMesh/Core/System/omstream.hh>
#include <OpenMesh/Core/Utils/Endian.hh>
#include <OpenMesh/Core/Utils/GenProg.hh>
#include <OpenMesh/Core/Utils/vector_traits.hh>
#include <OpenMesh/Core/Utils/BaseProperty.hh>
#include <OpenMesh/Core/Mesh/BaseKernel.hh>
#include <OpenMesh/Core/Mesh/AttribKernelT.hh>
// --------------------
#include <iostream>
#include <memory>
#if defined(OM_CC_GCC) && (OM_GCC_VERSION < 30000)
#include <OpenMesh/Tools/Utils/NumLimitsT.hh>
#define OM_MISSING_HEADER_LIMITS 1
#else
#include <limits>
#endif


//== NAMESPACES ==============================================================

#ifndef DOXY_IGNORE_THIS
namespace OpenMesh
{
namespace IO
{
namespace OM2Format
{
//=== IMPLEMENTATION ==========================================================


/** \name Mesh Reading / Writing
 */
//@{

//-----------------------------------------------------------------------------

// <:OM2 Header>
// AttributeDefinition 0
// AttributeDefinition 1
// <:OM Header>
// <:OM Payload>

// NOTICE!
//
// The usage of data types who differ in size
// on different pc architectures (32/64 bit) and/or
// operating systems, e.g. (unsigned) long, size_t,
// is not recommended because of inconsistencies
// in case of cross writing and reading.


struct BasePropertyType {
    BasePropertyType(std::string const &_name) : name(_name) {}
    virtual ~BasePropertyType();
    virtual bool fits(OpenMesh::BaseProperty const &prop) = 0;

    virtual void addToVertex(std::string const &name,
                             OpenMesh::BaseKernel &kern) = 0;
    virtual void addToHalfedge(std::string const &name,
                               OpenMesh::BaseKernel &kern) = 0;
    virtual void addToEdge(std::string const &name,
                           OpenMesh::BaseKernel &kern) = 0;
    virtual void addToFace(std::string const &name,
                           OpenMesh::BaseKernel &kern) = 0;
    virtual void addToMesh(std::string const &name,
                           OpenMesh::BaseKernel &kern) = 0;

    const std::string name;
};

template <class T> struct PropertyTypeT final : public BasePropertyType {
    PropertyTypeT(std::string const &_name) : BasePropertyType(_name) {}
    ~PropertyTypeT() {}

    bool fits(OpenMesh::BaseProperty const &prop) {
        return dynamic_cast<OpenMesh::PropertyT<T> const *>(&prop);
    }

    void addToVertex(std::string const &name,
                     OpenMesh::BaseKernel &kern) override {
        OpenMesh::VPropHandleT<T> handle;
        if (!kern.get_property_handle(handle, name))
            kern.add_property(handle, name);
        kern.property(handle).set_persistent(true);
    }
    void addToHalfedge(std::string const &name,
                       OpenMesh::BaseKernel &kern) override {
        OpenMesh::HPropHandleT<T> handle;
        if (!kern.get_property_handle(handle, name))
            kern.add_property(handle, name);
        kern.property(handle).set_persistent(true);
    }
    void addToEdge(std::string const &name,
                   OpenMesh::BaseKernel &kern) override {
        OpenMesh::EPropHandleT<T> handle;
        if (!kern.get_property_handle(handle, name))
            kern.add_property(handle, name);
        kern.property(handle).set_persistent(true);
    }
    void addToFace(std::string const &name,
                   OpenMesh::BaseKernel &kern) override {
        OpenMesh::FPropHandleT<T> handle;
        if (!kern.get_property_handle(handle, name))
            kern.add_property(handle, name);
        kern.property(handle).set_persistent(true);
    }
    void addToMesh(std::string const &name,
                   OpenMesh::BaseKernel &kern) override {
        OpenMesh::MPropHandleT<T> handle;
        if (!kern.get_property_handle(handle, name))
            kern.add_property(handle, name);
        kern.mproperty(handle).set_persistent(true);
    }
};

namespace TypeInfo {
    extern std::vector<std::unique_ptr<BasePropertyType>> sTypes;

    template <class T> void registerType(std::string const &identifier) {
        sTypes.push_back({new PropertyTypeT<T>(identifier)});
    }

    template <class T> BasePropertyType *get() {
        for (auto const &ptr : sTypes) {
            auto *p = dynamic_cast<PropertyTypeT<T> *>(ptr.get());
            if (p) return p;
        }
        return nullptr;
    }

    inline BasePropertyType *get(OpenMesh::BaseProperty const &prop) {
        for (auto const &ptr : sTypes) {
            if (ptr->fits(prop)) return ptr.get();
        }
        return nullptr;
    }

    inline BasePropertyType *get(std::string const &name) {
        for (auto const &ptr : sTypes) {
            if (ptr->name == name) return ptr.get();
        }
        return nullptr;
    }

    /// Identify exportable built-in properties and write the preamble.
    /// Use the returned Openmesh::IO::Options for the actual mesh export
    template <class MeshItems, class Connectivity>
    OpenMesh::IO::Options writePreambleAndBuiltins(
        std::ostream &os,
        OpenMesh::AttribKernelT<MeshItems, Connectivity> const &mesh);

    void writePreamble(std::ostream &os, OpenMesh::BaseKernel const &mesh,
                       OpenMesh::IO::Options opts = 0);

    /// Read the preamble and request the included properties on the mesh.
    /// Use the returned OpenMesh::IO::Options for the actual mesh import
    template <class MeshItems, class Connectivity>
    OpenMesh::IO::Options readPreambleAndBuiltins(
        std::istream &is,
        OpenMesh::AttribKernelT<MeshItems, Connectivity> &mesh);

    OpenMesh::IO::Options readPreamble(std::istream &is,
                                       OpenMesh::BaseKernel &mesh);

    bool checkMagic(char const * first4Bytes);
}
} // namespace OMFormat
//=============================================================================
} // namespace IO
} // namespace OpenMesh

#define OM2_REGISTER_PROPERTY_TYPE(type) OpenMesh::IO::OM2Format::type_info::registerType<type>(#type);

#endif
//=============================================================================
#if defined(OM_MISSING_HEADER_LIMITS)
#undef OM_MISSING_HEADER_LIMITS
#endif
//=============================================================================
#if defined(OM_INCLUDE_TEMPLATES) && !defined(OPENMESH_IO_OM2FORMAT_CC)
#define OPENMESH_IO_OM2FORMAT_TEMPLATES
#include "OM2FormatT.cc"
#endif
//=============================================================================
#endif
//=============================================================================
