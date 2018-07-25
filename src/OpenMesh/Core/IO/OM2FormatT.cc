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


//=============================================================================
//
//  Helper Functions for binary reading / writing
//
//=============================================================================

//== INCLUDES =================================================================

#include <OpenMesh/Core/IO/OM2Format.hh>

//== NAMESPACES ===============================================================

namespace OpenMesh {
namespace IO {
namespace OMFormat {

//== IMPLEMENTATION ===========================================================

namespace TypeInfo {

template <class MeshItems, class Connectivity>
OpenMesh::IO::Options readPreambleAndBuiltins(
    std::istream &is,
    OpenMesh::AttribKernelT<MeshItems, Connectivity> &mesh) {

    using Opts = OpenMesh::IO::Options;
    Opts props = readPreamble(is, mesh);
    if (props.vertex_has_normal()) mesh.request_vertex_normals();
    if (props.vertex_has_color()) mesh.request_vertex_colors();

    return props;
}

template <class MeshItems, class Connectivity>
OpenMesh::IO::Options writePreambleAndBuiltins(
    std::ostream &os,
    OpenMesh::AttribKernelT<MeshItems, Connectivity> const &mesh) {

    using Opts = OpenMesh::IO::Options;
    Opts opts;
    if (mesh.has_vertex_normals()) opts += Opts::VertexNormal;
    if (mesh.has_vertex_colors()) opts += Opts::VertexColor;
    if (mesh.has_vertex_texcoords2D()) opts += Opts::VertexTexCoord;

    writePreamble(os, mesh, opts);
    return opts;
}

}

} // namespace OMFormat
  // --------------------------------------------------------------------------

//=============================================================================
} // namespace IO
} // namespace OpenMesh
//=============================================================================
