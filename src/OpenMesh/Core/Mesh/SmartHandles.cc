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

SmartHalfedgeHandle SmartVertexHandle::out()
{
  assert(mesh() != nullptr);
  return make_smart(mesh()->halfedge_handle(*this), mesh());
}

SmartHalfedgeHandle SmartVertexHandle::halfedge()
{
  return out();
}

SmartHalfedgeHandle SmartVertexHandle::in()
{
  return out().opp();
}

SmartHalfedgeHandle SmartHalfedgeHandle::next()
{
  assert(mesh() != nullptr);
  return make_smart(mesh()->next_halfedge_handle(*this), mesh());
}

SmartHalfedgeHandle SmartHalfedgeHandle::prev()
{
  assert(mesh() != nullptr);
  return make_smart(mesh()->prev_halfedge_handle(*this), mesh());
}

SmartHalfedgeHandle SmartHalfedgeHandle::opp()
{
  assert(mesh() != nullptr);
  return make_smart(mesh()->opposite_halfedge_handle(*this), mesh());
}

SmartVertexHandle SmartHalfedgeHandle::to()
{
  assert(mesh() != nullptr);
  return make_smart(mesh()->to_vertex_handle(*this), mesh());
}

SmartVertexHandle SmartHalfedgeHandle::from()
{
  assert(mesh() != nullptr);
  return make_smart(mesh()->from_vertex_handle(*this), mesh());
}

SmartFaceHandle SmartHalfedgeHandle::face()
{
  assert(mesh() != nullptr);
  return make_smart(mesh()->face_handle(*this), mesh());
}

SmartHalfedgeHandle SmartEdgeHandle::h(unsigned int _i)
{
  assert(mesh() != nullptr);
  return make_smart(mesh()->halfedge_handle(*this, _i), mesh());
}

SmartHalfedgeHandle SmartEdgeHandle::h0()
{
  return h(0);
}

SmartHalfedgeHandle SmartEdgeHandle::h1()
{
  return h(1);
}

SmartVertexHandle SmartEdgeHandle::v(unsigned int _i)
{
  return h(_i).from();
}

SmartVertexHandle SmartEdgeHandle::v0()
{
  return v(0);
}

SmartVertexHandle SmartEdgeHandle::v1()
{
  return v(1);
}

SmartHalfedgeHandle SmartFaceHandle::halfedge()
{
  assert(mesh() != nullptr);
  return make_smart(mesh()->halfedge_handle(*this), mesh());
}


}

//=============================================================================
