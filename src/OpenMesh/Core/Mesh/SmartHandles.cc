/* ========================================================================= *
 *                                                                           *
 *                               OpenMesh                                    *
 *           Copyright (c) 2001-2019, RWTH-Aachen University                 *
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


//== INCLUDES =================================================================

#include "SmartHandles.hh"
#include <OpenMesh/Core/Mesh/PolyConnectivity.hh>

//== NAMESPACES ===============================================================

namespace OpenMesh
{

SmartHalfedgeHandle SmartVertexHandle::out() const
{
  assert(mesh() != nullptr);
  return make_smart(mesh()->halfedge_handle(*this), mesh());
}

SmartHalfedgeHandle SmartVertexHandle::halfedge() const
{
  return out();
}

SmartHalfedgeHandle SmartVertexHandle::in() const
{
  return out().opp();
}

PolyConnectivity::ConstVertexFaceRange SmartVertexHandle::faces() const
{
  assert(mesh() != nullptr);
  return mesh()->vf_range(*this);
}

PolyConnectivity::ConstVertexEdgeRange SmartVertexHandle::edges() const
{
  assert(mesh() != nullptr);
  return mesh()->ve_range(*this);
}

PolyConnectivity::ConstVertexVertexRange SmartVertexHandle::vertices() const
{
  assert(mesh() != nullptr);
  return mesh()->vv_range(*this);
}

PolyConnectivity::ConstVertexIHalfedgeRange SmartVertexHandle::incoming_halfedges() const
{
  assert(mesh() != nullptr);
  return mesh()->vih_range(*this);
}

PolyConnectivity::ConstVertexOHalfedgeRange SmartVertexHandle::outgoing_halfedges() const
{
  assert(mesh() != nullptr);
  return mesh()->voh_range(*this);
}

uint SmartVertexHandle::valence() const
{
  assert(mesh() != nullptr);
  return mesh()->valence(*this);
}

bool SmartVertexHandle::is_boundary() const
{
  assert(mesh() != nullptr);
  return mesh()->is_boundary(*this);
}

bool SmartVertexHandle::is_manifold() const
{
  assert(mesh() != nullptr);
  return mesh()->is_manifold(*this);
}

SmartHalfedgeHandle SmartHalfedgeHandle::next() const
{
  assert(mesh() != nullptr);
  return make_smart(mesh()->next_halfedge_handle(*this), mesh());
}

SmartHalfedgeHandle SmartHalfedgeHandle::prev() const
{
  assert(mesh() != nullptr);
  return make_smart(mesh()->prev_halfedge_handle(*this), mesh());
}

SmartHalfedgeHandle SmartHalfedgeHandle::opp() const
{
  assert(mesh() != nullptr);
  return make_smart(mesh()->opposite_halfedge_handle(*this), mesh());
}

SmartVertexHandle SmartHalfedgeHandle::to() const
{
  assert(mesh() != nullptr);
  return make_smart(mesh()->to_vertex_handle(*this), mesh());
}

SmartVertexHandle SmartHalfedgeHandle::from() const
{
  assert(mesh() != nullptr);
  return make_smart(mesh()->from_vertex_handle(*this), mesh());
}

SmartFaceHandle SmartHalfedgeHandle::face() const
{
  assert(mesh() != nullptr);
  return make_smart(mesh()->face_handle(*this), mesh());
}

bool SmartHalfedgeHandle::is_boundary() const
{
  assert(mesh() != nullptr);
  return mesh()->is_boundary(*this);
}

SmartHalfedgeHandle SmartEdgeHandle::halfedge(unsigned int _i) const
{
  assert(mesh() != nullptr);
  return make_smart(mesh()->halfedge_handle(*this, _i), mesh());
}

SmartHalfedgeHandle SmartEdgeHandle::h(unsigned int _i) const
{
  return halfedge(_i);
}

SmartHalfedgeHandle SmartEdgeHandle::h0() const
{
  return h(0);
}

SmartHalfedgeHandle SmartEdgeHandle::h1() const
{
  return h(1);
}

SmartVertexHandle SmartEdgeHandle::vertex(unsigned int _i) const
{
  return halfedge(_i).from();
}

SmartVertexHandle SmartEdgeHandle::v(unsigned int _i) const
{
  return vertex(_i);
}

SmartVertexHandle SmartEdgeHandle::v0() const
{
  return v(0);
}

SmartVertexHandle SmartEdgeHandle::v1() const
{
  return v(1);
}

bool SmartEdgeHandle::is_boundary() const
{
  assert(mesh() != nullptr);
  return mesh()->is_boundary(*this);
}

SmartHalfedgeHandle SmartFaceHandle::halfedge() const
{
  assert(mesh() != nullptr);
  return make_smart(mesh()->halfedge_handle(*this), mesh());
}

PolyConnectivity::ConstFaceVertexRange SmartFaceHandle::vertices() const
{
  assert(mesh() != nullptr);
  return mesh()->fv_range(*this);
}

PolyConnectivity::ConstFaceHalfedgeRange SmartFaceHandle::halfedges() const
{
  assert(mesh() != nullptr);
  return mesh()->fh_range(*this);
}

PolyConnectivity::ConstFaceEdgeRange SmartFaceHandle::edges() const
{
  assert(mesh() != nullptr);
  return mesh()->fe_range(*this);
}

PolyConnectivity::ConstFaceFaceRange SmartFaceHandle::faces() const
{
  assert(mesh() != nullptr);
  return mesh()->ff_range(*this);
}

uint SmartFaceHandle::valence() const
{
  assert(mesh() != nullptr);
  return mesh()->valence(*this);
}

bool SmartFaceHandle::is_boundary() const
{
  assert(mesh() != nullptr);
  return mesh()->is_boundary(*this);
}


}

//=============================================================================
