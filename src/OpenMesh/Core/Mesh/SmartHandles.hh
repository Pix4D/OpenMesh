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


#ifndef OPENMESH_SMARTHANDLES_HH
#define OPENMESH_SMARTHANDLES_HH


//== INCLUDES =================================================================

#include "Handles.hh"
#include <OpenMesh/Core/Mesh/PolyConnectivity.hh>


//== NAMESPACES ===============================================================

namespace OpenMesh {

//== FORWARD DECLARATION ======================================================

struct SmartVertexHandle;
struct SmartHalfedgeHandle;
struct SmartEdgeHandle;
struct SmartFaceHandle;


//== CLASS DEFINITION =========================================================

class SmartBaseHandle
{
public:
  explicit SmartBaseHandle(const PolyConnectivity* _mesh = nullptr) : mesh_(_mesh) {}

  /// Get the underlying mesh of this handle
  const PolyConnectivity* mesh() const { return mesh_; }

  // TODO: should operators ==, !=, < look at mesh_?

private:
  const PolyConnectivity* mesh_;

};

struct SmartVertexHandle : public SmartBaseHandle, VertexHandle
{
  explicit SmartVertexHandle(int _idx=-1, const PolyConnectivity* _mesh = nullptr) : SmartBaseHandle(_mesh), VertexHandle(_idx) {}

  SmartHalfedgeHandle out()      const;
  SmartHalfedgeHandle halfedge() const; // alias for out
  SmartHalfedgeHandle in()       const;

  PolyConnectivity::ConstVertexFaceRange      faces()              const;
  PolyConnectivity::ConstVertexEdgeRange      edges()              const;
  PolyConnectivity::ConstVertexVertexRange    vertices()           const;
  PolyConnectivity::ConstVertexIHalfedgeRange incoming_halfedges() const;
  PolyConnectivity::ConstVertexOHalfedgeRange outgoing_halfedges() const;

  uint valence()     const;
  bool is_boundary() const;
  bool is_manifold() const;
};

struct SmartHalfedgeHandle : public SmartBaseHandle, HalfedgeHandle
{
  explicit SmartHalfedgeHandle(int _idx=-1, const PolyConnectivity* _mesh = nullptr) : SmartBaseHandle(_mesh), HalfedgeHandle(_idx) {}

  SmartHalfedgeHandle next() const;
  SmartHalfedgeHandle prev() const;
  SmartHalfedgeHandle opp()  const;
  SmartVertexHandle   to()   const;
  SmartVertexHandle   from() const;
  SmartFaceHandle     face() const;

  bool is_boundary() const;
};

struct SmartEdgeHandle : public SmartBaseHandle, EdgeHandle
{
  explicit SmartEdgeHandle(int _idx=-1, const PolyConnectivity* _mesh = nullptr) : SmartBaseHandle(_mesh), EdgeHandle(_idx) {}

  SmartHalfedgeHandle halfedge(unsigned int _i) const;
  SmartHalfedgeHandle h(unsigned int _i)        const;
  SmartHalfedgeHandle h0()                      const;
  SmartHalfedgeHandle h1()                      const;
  SmartVertexHandle   vertex(unsigned int _i)   const;
  SmartVertexHandle   v(unsigned int _i)        const;
  SmartVertexHandle   v0()                      const;
  SmartVertexHandle   v1()                      const;

  bool is_boundary() const;
};

struct SmartFaceHandle : public SmartBaseHandle, FaceHandle
{
  explicit SmartFaceHandle(int _idx=-1, const PolyConnectivity* _mesh = nullptr) : SmartBaseHandle(_mesh), FaceHandle(_idx) {}

  SmartHalfedgeHandle halfedge() const;

  PolyConnectivity::ConstFaceVertexRange   vertices()  const;
  PolyConnectivity::ConstFaceHalfedgeRange halfedges() const;
  PolyConnectivity::ConstFaceEdgeRange     edges()     const;
  PolyConnectivity::ConstFaceFaceRange     faces()     const;

  uint valence()     const;
  bool is_boundary() const;
};


inline SmartVertexHandle   make_smart(VertexHandle   _vh, const PolyConnectivity* _mesh) { return SmartVertexHandle  (_vh.idx(), _mesh); }
inline SmartHalfedgeHandle make_smart(HalfedgeHandle _hh, const PolyConnectivity* _mesh) { return SmartHalfedgeHandle(_hh.idx(), _mesh); }
inline SmartEdgeHandle     make_smart(EdgeHandle     _eh, const PolyConnectivity* _mesh) { return SmartEdgeHandle    (_eh.idx(), _mesh); }
inline SmartFaceHandle     make_smart(FaceHandle     _fh, const PolyConnectivity* _mesh) { return SmartFaceHandle    (_fh.idx(), _mesh); }

inline SmartVertexHandle   make_smart(VertexHandle   _vh, const PolyConnectivity& _mesh) { return SmartVertexHandle  (_vh.idx(), &_mesh); }
inline SmartHalfedgeHandle make_smart(HalfedgeHandle _hh, const PolyConnectivity& _mesh) { return SmartHalfedgeHandle(_hh.idx(), &_mesh); }
inline SmartEdgeHandle     make_smart(EdgeHandle     _eh, const PolyConnectivity& _mesh) { return SmartEdgeHandle    (_eh.idx(), &_mesh); }
inline SmartFaceHandle     make_smart(FaceHandle     _fh, const PolyConnectivity& _mesh) { return SmartFaceHandle    (_fh.idx(), &_mesh); }


//=============================================================================
} // namespace OpenMesh
//=============================================================================


#endif // OPENMESH_SMARTHANDLES_HH
//=============================================================================
