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



#ifndef OPENMESH_POLYCONNECTIVITY_HH
#define OPENMESH_POLYCONNECTIVITY_HH

#include <OpenMesh/Core/Mesh/ArrayKernel.hh>
#include <OpenMesh/Core/Mesh/SmartRange.hh>

namespace OpenMesh
{

namespace Iterators
{
  template <class Mesh, class ValueHandle, class MemberOwner, bool (MemberOwner::*PrimitiveStatusMember)() const, size_t (MemberOwner::*PrimitiveCountMember)() const>
  class GenericIteratorT;

  template<class Mesh>
  class GenericCirculatorBaseT;

  template<typename Traits>
  class GenericCirculatorT_DEPRECATED;

  template<typename Traits, bool CW>
  class GenericCirculatorT;
}

template <typename RangeTraitT>
class EntityRange;

template<
  typename CONTAINER_T,
  typename ITER_T,
  ITER_T (CONTAINER_T::*begin_fn)() const,
  ITER_T (CONTAINER_T::*end_fn)() const>
struct RangeTraitT
{
  using CONTAINER_TYPE = CONTAINER_T;
  using ITER_TYPE = ITER_T;
  static ITER_TYPE begin(const CONTAINER_TYPE& _container) { return (_container.*begin_fn)(); }
  static ITER_TYPE end(const CONTAINER_TYPE& _container)   { return (_container.*end_fn)(); }
};


template <typename CirculatorRangeTraitT>
class CirculatorRange;

template<
    typename CONTAINER_T,
    typename ITER_T,
    typename CENTER_ENTITY_T,
    typename TO_ENTITY_T,
    ITER_T (CONTAINER_T::*begin_fn)(CENTER_ENTITY_T) const,
    ITER_T (CONTAINER_T::*end_fn)(CENTER_ENTITY_T) const>
struct CirculatorRangeTraitT
{
  using CONTAINER_TYPE = CONTAINER_T;
  using ITER_TYPE = ITER_T;
  using CENTER_ENTITY_TYPE = CENTER_ENTITY_T;
  using TO_ENTITYE_TYPE = TO_ENTITY_T;
  static ITER_TYPE begin(const CONTAINER_TYPE& _container, CENTER_ENTITY_TYPE _ce) { return (_container.*begin_fn)(_ce); }
  static ITER_TYPE end(const CONTAINER_TYPE& _container, CENTER_ENTITY_TYPE _ce)   { return (_container.*end_fn)(_ce); }
};



/** \brief Connectivity Class for polygonal meshes
*/
class OPENMESHDLLEXPORT PolyConnectivity : public ArrayKernel
{
public:
  /// \name Mesh Handles
  //@{
  /// Invalid handle
  static const VertexHandle                           InvalidVertexHandle;
  /// Invalid handle
  static const HalfedgeHandle                         InvalidHalfedgeHandle;
  /// Invalid handle
  static const EdgeHandle                             InvalidEdgeHandle;
  /// Invalid handle
  static const FaceHandle                             InvalidFaceHandle;
  //@}

  typedef PolyConnectivity                            This;

  //--- iterators ---

  /** \name Mesh Iterators
      Refer to OpenMesh::Mesh::Iterators or \ref mesh_iterators for
      documentation.
  */
  //@{
  /// Linear iterator
  typedef Iterators::GenericIteratorT<This, This::VertexHandle, ArrayKernel , &ArrayKernel::has_vertex_status, &ArrayKernel::n_vertices> VertexIter;
  typedef Iterators::GenericIteratorT<This, This::HalfedgeHandle, ArrayKernel , &ArrayKernel::has_halfedge_status, &ArrayKernel::n_halfedges> HalfedgeIter;
  typedef Iterators::GenericIteratorT<This, This::EdgeHandle, ArrayKernel , &ArrayKernel::has_edge_status, &ArrayKernel::n_edges> EdgeIter;
  typedef Iterators::GenericIteratorT<This, This::FaceHandle, ArrayKernel , &ArrayKernel::has_face_status, &ArrayKernel::n_faces> FaceIter;

  typedef VertexIter ConstVertexIter;
  typedef HalfedgeIter ConstHalfedgeIter;
  typedef EdgeIter ConstEdgeIter;
  typedef FaceIter ConstFaceIter;
  //@}

  //--- circulators ---

  /** \name Mesh Circulators
      Refer to OpenMesh::Mesh::Iterators or \ref mesh_iterators
      for documentation.
  */
  //@{

  /*
   * Vertex-centered circulators
   */

  struct VertexVertexTraits
  {
    using Mesh = This;
    using CenterEntityHandle = This::VertexHandle;
    using ValueHandle = This::VertexHandle;
    static ValueHandle toHandle(const Mesh* const _mesh, This::HalfedgeHandle _heh) { return _mesh->to_vertex_handle(_heh);}
  };


  /**
   * Enumerates 1-ring vertices in a clockwise fashion.
   */
  typedef Iterators::GenericCirculatorT_DEPRECATED<VertexVertexTraits> VertexVertexIter;
  typedef Iterators::GenericCirculatorT<VertexVertexTraits, true> VertexVertexCWIter;

  /**
   * Enumerates 1-ring vertices in a counter clockwise fashion.
   */
  typedef Iterators::GenericCirculatorT<VertexVertexTraits, false>  VertexVertexCCWIter;


  struct VertexHalfedgeTraits
  {
    using Mesh = This;
    using CenterEntityHandle = This::VertexHandle;
    using ValueHandle = This::HalfedgeHandle;
    static ValueHandle toHandle(const Mesh* const _mesh, This::HalfedgeHandle _heh) { return _heh;}
  };

  /**
   * Enumerates outgoing half edges in a clockwise fashion.
   */
  typedef Iterators::GenericCirculatorT_DEPRECATED<VertexHalfedgeTraits> VertexOHalfedgeIter;
  typedef Iterators::GenericCirculatorT<VertexHalfedgeTraits, true> VertexOHalfedgeCWIter;

  /**
   * Enumerates outgoing half edges in a counter clockwise fashion.
   */
  typedef Iterators::GenericCirculatorT<VertexHalfedgeTraits, false> VertexOHalfedgeCCWIter;

  struct VertexOppositeHalfedgeTraits
  {
    using Mesh = This;
    using CenterEntityHandle = This::VertexHandle;
    using ValueHandle = This::HalfedgeHandle;
    static ValueHandle toHandle(const Mesh* const _mesh, This::HalfedgeHandle _heh) { return _mesh->opposite_halfedge_handle(_heh); }
  };

  /**
   * Enumerates incoming half edges in a clockwise fashion.
   */
  typedef Iterators::GenericCirculatorT_DEPRECATED<VertexOppositeHalfedgeTraits> VertexIHalfedgeIter;
  typedef Iterators::GenericCirculatorT<VertexOppositeHalfedgeTraits, true> VertexIHalfedgeCWIter;

  /**
   * Enumerates incoming half edges in a counter clockwise fashion.
   */
  typedef Iterators::GenericCirculatorT<VertexOppositeHalfedgeTraits, false> VertexIHalfedgeCCWIter;


  struct VertexFaceTraits
  {
    using Mesh = This;
    using CenterEntityHandle = This::VertexHandle;
    using ValueHandle = This::FaceHandle;
    static ValueHandle toHandle(const Mesh* const _mesh, This::HalfedgeHandle _heh) { return _mesh->face_handle(_heh); }
  };

  /**
   * Enumerates incident faces in a clockwise fashion.
   */
  typedef Iterators::GenericCirculatorT_DEPRECATED<VertexFaceTraits> VertexFaceIter;
  typedef Iterators::GenericCirculatorT<VertexFaceTraits, true> VertexFaceCWIter;

  /**
   * Enumerates incident faces in a counter clockwise fashion.
   */
  typedef Iterators::GenericCirculatorT<VertexFaceTraits, false> VertexFaceCCWIter;


  struct VertexEdgeTraits
  {
    using Mesh = This;
    using CenterEntityHandle = This::VertexHandle;
    using ValueHandle = This::EdgeHandle;
    static ValueHandle toHandle(const Mesh* const _mesh, This::HalfedgeHandle _heh) { return _mesh->edge_handle(_heh); }
  };

  /**
   * Enumerates incident edges in a clockwise fashion.
   */
  typedef Iterators::GenericCirculatorT_DEPRECATED<VertexEdgeTraits> VertexEdgeIter;
  typedef Iterators::GenericCirculatorT<VertexEdgeTraits, true> VertexEdgeCWIter;
  /**
   * Enumerates incident edges in a counter clockwise fashion.
   */
  typedef Iterators::GenericCirculatorT<VertexEdgeTraits, false> VertexEdgeCCWIter;


  struct FaceHalfedgeTraits
  {
    using Mesh = This;
    using CenterEntityHandle = This::FaceHandle;
    using ValueHandle = This::HalfedgeHandle;
    static ValueHandle toHandle(const Mesh* const _mesh, This::HalfedgeHandle _heh) { return _heh; }
  };

  /**
   * Identical to #FaceHalfedgeIter. God knows why this typedef exists.
   */
  typedef Iterators::GenericCirculatorT_DEPRECATED<FaceHalfedgeTraits> HalfedgeLoopIter;
  typedef Iterators::GenericCirculatorT<FaceHalfedgeTraits, false> HalfedgeLoopCWIter;
  /**
   * Identical to #FaceHalfedgeIter. God knows why this typedef exists.
   */
  typedef Iterators::GenericCirculatorT<FaceHalfedgeTraits, true> HalfedgeLoopCCWIter;

  typedef VertexVertexIter        ConstVertexVertexIter;
  typedef VertexVertexCWIter      ConstVertexVertexCWIter;
  typedef VertexVertexCCWIter     ConstVertexVertexCCWIter;
  typedef VertexOHalfedgeIter     ConstVertexOHalfedgeIter;
  typedef VertexOHalfedgeCWIter   ConstVertexOHalfedgeCWIter;
  typedef VertexOHalfedgeCCWIter  ConstVertexOHalfedgeCCWIter;
  typedef VertexIHalfedgeIter     ConstVertexIHalfedgeIter;
  typedef VertexIHalfedgeCWIter   ConstVertexIHalfedgeCWIter;
  typedef VertexIHalfedgeCCWIter  ConstVertexIHalfedgeCCWIter;
  typedef VertexFaceIter          ConstVertexFaceIter;
  typedef VertexFaceCWIter        ConstVertexFaceCWIter;
  typedef VertexFaceCCWIter       ConstVertexFaceCCWIter;
  typedef VertexEdgeIter          ConstVertexEdgeIter;
  typedef VertexEdgeCWIter        ConstVertexEdgeCWIter;
  typedef VertexEdgeCCWIter       ConstVertexEdgeCCWIter;

  /*
   * Face-centered circulators
   */

  struct FaceVertexTraits
  {
    using Mesh = This;
    using CenterEntityHandle = This::FaceHandle;
    using ValueHandle = This::VertexHandle;
    static ValueHandle toHandle(const Mesh* const _mesh, This::HalfedgeHandle _heh) { return _mesh->to_vertex_handle(_heh); }
  };

  /**
   * Enumerate incident vertices in a counter clockwise fashion.
   */
  typedef Iterators::GenericCirculatorT_DEPRECATED<FaceVertexTraits> FaceVertexIter;
  typedef Iterators::GenericCirculatorT<FaceVertexTraits, true> FaceVertexCCWIter;

  /**
   * Enumerate incident vertices in a clockwise fashion.
   */
  typedef Iterators::GenericCirculatorT<FaceVertexTraits, false> FaceVertexCWIter;

  /**
   * Enumerate incident half edges in a counter clockwise fashion.
   */
  typedef Iterators::GenericCirculatorT_DEPRECATED<FaceHalfedgeTraits> FaceHalfedgeIter;
  typedef Iterators::GenericCirculatorT<FaceHalfedgeTraits, true> FaceHalfedgeCCWIter;

  /**
   * Enumerate incident half edges in a clockwise fashion.
   */
  typedef Iterators::GenericCirculatorT<FaceHalfedgeTraits, false> FaceHalfedgeCWIter;


  struct FaceEdgeTraits
  {
    using Mesh = This;
    using CenterEntityHandle = This::FaceHandle;
    using ValueHandle = This::EdgeHandle;
    static ValueHandle toHandle(const Mesh* const _mesh, This::HalfedgeHandle _heh) { return _mesh->edge_handle(_heh); }
  };

  /**
   * Enumerate incident edges in a counter clockwise fashion.
   */
  typedef Iterators::GenericCirculatorT_DEPRECATED<FaceEdgeTraits> FaceEdgeIter;
  typedef Iterators::GenericCirculatorT<FaceEdgeTraits, true> FaceEdgeCCWIter;

  /**
   * Enumerate incident edges in a clockwise fashion.
   */
  typedef Iterators::GenericCirculatorT<FaceEdgeTraits, false> FaceEdgeCWIter;


  struct FaceFaceTraits
  {
    using Mesh = This;
    using CenterEntityHandle = This::FaceHandle;
    using ValueHandle = This::FaceHandle;
    static ValueHandle toHandle(const Mesh* const _mesh, This::HalfedgeHandle _heh) { return _mesh->face_handle(_mesh->opposite_halfedge_handle(_heh)); }
  };

  /**
   * Enumerate adjacent faces in a counter clockwise fashion.
   */
  typedef Iterators::GenericCirculatorT_DEPRECATED<FaceFaceTraits> FaceFaceIter;
  typedef Iterators::GenericCirculatorT<FaceFaceTraits, true> FaceFaceCCWIter;

  /**
   * Enumerate adjacent faces in a clockwise fashion.
   */
  typedef Iterators::GenericCirculatorT<FaceFaceTraits, false> FaceFaceCWIter;

  typedef FaceVertexIter        ConstFaceVertexIter;
  typedef FaceVertexCWIter      ConstFaceVertexCWIter;
  typedef FaceVertexCCWIter     ConstFaceVertexCCWIter;
  typedef FaceHalfedgeIter      ConstFaceHalfedgeIter;
  typedef FaceHalfedgeCWIter    ConstFaceHalfedgeCWIter;
  typedef FaceHalfedgeCCWIter   ConstFaceHalfedgeCCWIter;
  typedef FaceEdgeIter          ConstFaceEdgeIter;
  typedef FaceEdgeCWIter        ConstFaceEdgeCWIter;
  typedef FaceEdgeCCWIter       ConstFaceEdgeCCWIter;
  typedef FaceFaceIter          ConstFaceFaceIter;
  typedef FaceFaceCWIter        ConstFaceFaceCWIter;
  typedef FaceFaceCCWIter       ConstFaceFaceCCWIter;

  /*
   * Halfedge circulator
   */
  typedef HalfedgeLoopIter      ConstHalfedgeLoopIter;
  typedef HalfedgeLoopCWIter    ConstHalfedgeLoopCWIter;
  typedef HalfedgeLoopCCWIter   ConstHalfedgeLoopCCWIter;

  //@}

  // --- shortcuts

  /** \name Typedef Shortcuts
      Provided for convenience only
  */
  //@{
  /// Alias typedef
  typedef VertexHandle    VHandle;
  typedef HalfedgeHandle  HHandle;
  typedef EdgeHandle      EHandle;
  typedef FaceHandle      FHandle;

  typedef VertexIter    VIter;
  typedef HalfedgeIter  HIter;
  typedef EdgeIter      EIter;
  typedef FaceIter      FIter;

  typedef ConstVertexIter    CVIter;
  typedef ConstHalfedgeIter  CHIter;
  typedef ConstEdgeIter      CEIter;
  typedef ConstFaceIter      CFIter;

  typedef VertexVertexIter        VVIter;
  typedef VertexVertexCWIter      VVCWIter;
  typedef VertexVertexCCWIter     VVCCWIter;
  typedef VertexOHalfedgeIter     VOHIter;
  typedef VertexOHalfedgeCWIter   VOHCWIter;
  typedef VertexOHalfedgeCCWIter  VOHCCWIter;
  typedef VertexIHalfedgeIter     VIHIter;
  typedef VertexIHalfedgeCWIter   VIHICWter;
  typedef VertexIHalfedgeCCWIter  VIHICCWter;
  typedef VertexEdgeIter          VEIter;
  typedef VertexEdgeCWIter        VECWIter;
  typedef VertexEdgeCCWIter       VECCWIter;
  typedef VertexFaceIter          VFIter;
  typedef VertexFaceCWIter        VFCWIter;
  typedef VertexFaceCCWIter       VFCCWIter;
  typedef FaceVertexIter          FVIter;
  typedef FaceVertexCWIter        FVCWIter;
  typedef FaceVertexCCWIter       FVCCWIter;
  typedef FaceHalfedgeIter        FHIter;
  typedef FaceHalfedgeCWIter      FHCWIter;
  typedef FaceHalfedgeCCWIter     FHCWWIter;
  typedef FaceEdgeIter            FEIter;
  typedef FaceEdgeCWIter          FECWIter;
  typedef FaceEdgeCCWIter         FECWWIter;
  typedef FaceFaceIter            FFIter;

  typedef ConstVertexVertexIter         CVVIter;
  typedef ConstVertexVertexCWIter       CVVCWIter;
  typedef ConstVertexVertexCCWIter      CVVCCWIter;
  typedef ConstVertexOHalfedgeIter      CVOHIter;
  typedef ConstVertexOHalfedgeCWIter    CVOHCWIter;
  typedef ConstVertexOHalfedgeCCWIter   CVOHCCWIter;
  typedef ConstVertexIHalfedgeIter      CVIHIter;
  typedef ConstVertexIHalfedgeCWIter    CVIHCWIter;
  typedef ConstVertexIHalfedgeCCWIter   CVIHCCWIter;
  typedef ConstVertexEdgeIter           CVEIter;
  typedef ConstVertexEdgeCWIter         CVECWIter;
  typedef ConstVertexEdgeCCWIter        CVECCWIter;
  typedef ConstVertexFaceIter           CVFIter;
  typedef ConstVertexFaceCWIter         CVFCWIter;
  typedef ConstVertexFaceCCWIter        CVFCCWIter;
  typedef ConstFaceVertexIter           CFVIter;
  typedef ConstFaceVertexCWIter         CFVCWIter;
  typedef ConstFaceVertexCCWIter        CFVCCWIter;
  typedef ConstFaceHalfedgeIter         CFHIter;
  typedef ConstFaceHalfedgeCWIter       CFHCWIter;
  typedef ConstFaceHalfedgeCCWIter      CFHCCWIter;
  typedef ConstFaceEdgeIter             CFEIter;
  typedef ConstFaceEdgeCWIter           CFECWIter;
  typedef ConstFaceEdgeCCWIter          CFECCWIter;
  typedef ConstFaceFaceIter             CFFIter;
  typedef ConstFaceFaceCWIter           CFFCWIter;
  typedef ConstFaceFaceCCWIter          CFFCCWIter;
  //@}

public:

  PolyConnectivity()  {}
  virtual ~PolyConnectivity() {}

  inline static bool is_triangles()
  { return false; }

  /** assign_connectivity() method. See ArrayKernel::assign_connectivity()
      for more details. */
  inline void assign_connectivity(const PolyConnectivity& _other)
  { ArrayKernel::assign_connectivity(_other); }
  
  /** \name Adding items to a mesh
  */
  //@{

  /// Add a new vertex 
  inline VertexHandle add_vertex()
  { return new_vertex(); }

  /** \brief Add and connect a new face
  *
  * Create a new face consisting of the vertices provided by the vertex handle vector.
  * (The vertices have to be already added to the mesh by add_vertex)
  *
  * @param _vhandles sorted list of vertex handles (also defines order in which the vertices are added to the face)
  */
  FaceHandle add_face(const std::vector<VertexHandle>& _vhandles);
 
   
  /** \brief Add and connect a new face
  *
  * Create a new face consisting of three vertices provided by the handles.
  * (The vertices have to be already added to the mesh by add_vertex)
  *
  * @param _vh0 First  vertex handle
  * @param _vh1 Second vertex handle
  * @param _vh2 Third  vertex handle
  */
  FaceHandle add_face(VertexHandle _vh0, VertexHandle _vh1, VertexHandle _vh2);

  /** \brief Add and connect a new face
  *
  * Create a new face consisting of four vertices provided by the handles.
  * (The vertices have to be already added to the mesh by add_vertex)
  *
  * @param _vh0 First  vertex handle
  * @param _vh1 Second vertex handle
  * @param _vh2 Third  vertex handle
  * @param _vh3 Fourth vertex handle
  */
  FaceHandle add_face(VertexHandle _vh0, VertexHandle _vh1, VertexHandle _vh2, VertexHandle _vh3);
 
  /** \brief Add and connect a new face
  *
  * Create a new face consisting of vertices provided by a handle array.
  * (The vertices have to be already added to the mesh by add_vertex)
  *
  * @param _vhandles pointer to a sorted list of vertex handles (also defines order in which the vertices are added to the face)
  * @param _vhs_size number of vertex handles in the array
  */
  FaceHandle add_face(const VertexHandle* _vhandles, size_t _vhs_size);

  //@}

  /// \name Deleting mesh items and other connectivity/topology modifications
  //@{

  /** Returns whether collapsing halfedge _heh is ok or would lead to
      topological inconsistencies.
      \attention This method need the Attributes::Status attribute and
      changes the \em tagged bit.  */
  bool is_collapse_ok(HalfedgeHandle _he);
    
    
  /** Mark vertex and all incident edges and faces deleted.
      Items marked deleted will be removed by garbageCollection().
      \attention Needs the Attributes::Status attribute for vertices,
      edges and faces.
  */
  void delete_vertex(VertexHandle _vh, bool _delete_isolated_vertices = true);

  /** Mark edge (two opposite halfedges) and incident faces deleted.
      Resulting isolated vertices are marked deleted if
      _delete_isolated_vertices is true. Items marked deleted will be
      removed by garbageCollection().

      \attention Needs the Attributes::Status attribute for vertices,
      edges and faces.
  */
  void delete_edge(EdgeHandle _eh, bool _delete_isolated_vertices=true);

  /** Delete face _fh and resulting degenerated empty halfedges as
      well.  Resulting isolated vertices will be deleted if
      _delete_isolated_vertices is true.

      \attention All item will only be marked to be deleted. They will
      actually be removed by calling garbage_collection().

      \attention Needs the Attributes::Status attribute for vertices,
      edges and faces.
  */
  void delete_face(FaceHandle _fh, bool _delete_isolated_vertices=true);

  //@}
  
  /** \name Begin and end iterators
  */
  //@{

  /// Begin iterator for vertices
  VertexIter vertices_begin();
  /// Const begin iterator for vertices
  ConstVertexIter vertices_begin() const;
  /// End iterator for vertices
  VertexIter vertices_end();
  /// Const end iterator for vertices
  ConstVertexIter vertices_end() const;

  /// Begin iterator for halfedges
  HalfedgeIter halfedges_begin();
  /// Const begin iterator for halfedges
  ConstHalfedgeIter halfedges_begin() const;
  /// End iterator for halfedges
  HalfedgeIter halfedges_end();
  /// Const end iterator for halfedges
  ConstHalfedgeIter halfedges_end() const;

  /// Begin iterator for edges
  EdgeIter edges_begin();
  /// Const begin iterator for edges
  ConstEdgeIter edges_begin() const;
  /// End iterator for edges
  EdgeIter edges_end();
  /// Const end iterator for edges
  ConstEdgeIter edges_end() const;

  /// Begin iterator for faces
  FaceIter faces_begin();
  /// Const begin iterator for faces
  ConstFaceIter faces_begin() const;
  /// End iterator for faces
  FaceIter faces_end();
  /// Const end iterator for faces
  ConstFaceIter faces_end() const;
  //@}


  /** \name Begin for skipping iterators
  */
  //@{

  /// Begin iterator for vertices
  VertexIter vertices_sbegin();
  /// Const begin iterator for vertices
  ConstVertexIter vertices_sbegin() const;

  /// Begin iterator for halfedges
  HalfedgeIter halfedges_sbegin();
  /// Const begin iterator for halfedges
  ConstHalfedgeIter halfedges_sbegin() const;

  /// Begin iterator for edges
  EdgeIter edges_sbegin();
  /// Const begin iterator for edges
  ConstEdgeIter edges_sbegin() const;

  /// Begin iterator for faces
  FaceIter faces_sbegin();
  /// Const begin iterator for faces
  ConstFaceIter faces_sbegin() const;

  //@}

  //--- circulators ---

  /** \name Vertex and Face circulators
  */
  //@{

  /// vertex - vertex circulator
  VertexVertexIter vv_iter(VertexHandle _vh);
  /// vertex - vertex circulator cw
  VertexVertexCWIter vv_cwiter(VertexHandle _vh);
  /// vertex - vertex circulator ccw
  VertexVertexCCWIter vv_ccwiter(VertexHandle _vh);
  /// vertex - incoming halfedge circulator
  VertexIHalfedgeIter vih_iter(VertexHandle _vh);
  /// vertex - incoming halfedge circulator cw
  VertexIHalfedgeCWIter vih_cwiter(VertexHandle _vh);
  /// vertex - incoming halfedge circulator ccw
  VertexIHalfedgeCCWIter vih_ccwiter(VertexHandle _vh);
  /// vertex - outgoing halfedge circulator
  VertexOHalfedgeIter voh_iter(VertexHandle _vh);
  /// vertex - outgoing halfedge circulator cw
  VertexOHalfedgeCWIter voh_cwiter(VertexHandle _vh);
  /// vertex - outgoing halfedge circulator ccw
  VertexOHalfedgeCCWIter voh_ccwiter(VertexHandle _vh);
  /// vertex - edge circulator
  VertexEdgeIter ve_iter(VertexHandle _vh);
  /// vertex - edge circulator cw
  VertexEdgeCWIter ve_cwiter(VertexHandle _vh);
  /// vertex - edge circulator ccw
  VertexEdgeCCWIter ve_ccwiter(VertexHandle _vh);
  /// vertex - face circulator
  VertexFaceIter vf_iter(VertexHandle _vh);
  /// vertex - face circulator cw
  VertexFaceCWIter vf_cwiter(VertexHandle _vh);
  /// vertex - face circulator ccw
  VertexFaceCCWIter vf_ccwiter(VertexHandle _vh);

  /// const vertex circulator
  ConstVertexVertexIter cvv_iter(VertexHandle _vh) const;
  /// const vertex circulator cw
  ConstVertexVertexCWIter cvv_cwiter(VertexHandle _vh) const;
  /// const vertex circulator ccw
  ConstVertexVertexCCWIter cvv_ccwiter(VertexHandle _vh) const;
  /// const vertex - incoming halfedge circulator
  ConstVertexIHalfedgeIter cvih_iter(VertexHandle _vh) const;
  /// const vertex - incoming halfedge circulator cw
  ConstVertexIHalfedgeCWIter cvih_cwiter(VertexHandle _vh) const;
  /// const vertex - incoming halfedge circulator ccw
  ConstVertexIHalfedgeCCWIter cvih_ccwiter(VertexHandle _vh) const;
  /// const vertex - outgoing halfedge circulator
  ConstVertexOHalfedgeIter cvoh_iter(VertexHandle _vh) const;
  /// const vertex - outgoing halfedge circulator cw
  ConstVertexOHalfedgeCWIter cvoh_cwiter(VertexHandle _vh) const;
  /// const vertex - outgoing halfedge circulator ccw
  ConstVertexOHalfedgeCCWIter cvoh_ccwiter(VertexHandle _vh) const;
  /// const vertex - edge circulator
  ConstVertexEdgeIter cve_iter(VertexHandle _vh) const;
  /// const vertex - edge circulator cw
  ConstVertexEdgeCWIter cve_cwiter(VertexHandle _vh) const;
  /// const vertex - edge circulator ccw
  ConstVertexEdgeCCWIter cve_ccwiter(VertexHandle _vh) const;
  /// const vertex - face circulator
  ConstVertexFaceIter cvf_iter(VertexHandle _vh) const;
  /// const vertex - face circulator cw
  ConstVertexFaceCWIter cvf_cwiter(VertexHandle _vh) const;
  /// const vertex - face circulator ccw
  ConstVertexFaceCCWIter cvf_ccwiter(VertexHandle _vh) const;

  /// face - vertex circulator
  FaceVertexIter fv_iter(FaceHandle _fh);
  /// face - vertex circulator cw
  FaceVertexCWIter fv_cwiter(FaceHandle _fh);
  /// face - vertex circulator ccw
  FaceVertexCCWIter fv_ccwiter(FaceHandle _fh);
  /// face - halfedge circulator
  FaceHalfedgeIter fh_iter(FaceHandle _fh);
  /// face - halfedge circulator cw
  FaceHalfedgeCWIter fh_cwiter(FaceHandle _fh);
  /// face - halfedge circulator ccw
  FaceHalfedgeCCWIter fh_ccwiter(FaceHandle _fh);
  /// face - edge circulator
  FaceEdgeIter fe_iter(FaceHandle _fh);
  /// face - edge circulator cw
  FaceEdgeCWIter fe_cwiter(FaceHandle _fh);
  /// face - edge circulator ccw
  FaceEdgeCCWIter fe_ccwiter(FaceHandle _fh);
  /// face - face circulator
  FaceFaceIter ff_iter(FaceHandle _fh);
  /// face - face circulator cw
  FaceFaceCWIter ff_cwiter(FaceHandle _fh);
  /// face - face circulator ccw
  FaceFaceCCWIter ff_ccwiter(FaceHandle _fh);

  /// const face - vertex circulator
  ConstFaceVertexIter cfv_iter(FaceHandle _fh) const;
  /// const face - vertex circulator cw
  ConstFaceVertexCWIter cfv_cwiter(FaceHandle _fh) const;
  /// const face - vertex circulator ccw
  ConstFaceVertexCCWIter cfv_ccwiter(FaceHandle _fh) const;
  /// const face - halfedge circulator
  ConstFaceHalfedgeIter cfh_iter(FaceHandle _fh) const;
  /// const face - halfedge circulator cw
  ConstFaceHalfedgeCWIter cfh_cwiter(FaceHandle _fh) const;
  /// const face - halfedge circulator ccw
  ConstFaceHalfedgeCCWIter cfh_ccwiter(FaceHandle _fh) const;
  /// const face - edge circulator
  ConstFaceEdgeIter cfe_iter(FaceHandle _fh) const;
  /// const face - edge circulator cw
  ConstFaceEdgeCWIter cfe_cwiter(FaceHandle _fh) const;
  /// const face - edge circulator ccw
  ConstFaceEdgeCCWIter cfe_ccwiter(FaceHandle _fh) const;
  /// const face - face circulator
  ConstFaceFaceIter cff_iter(FaceHandle _fh) const;
  /// const face - face circulator cw
  ConstFaceFaceCWIter cff_cwiter(FaceHandle _fh) const;
  /// const face - face circulator
  ConstFaceFaceCCWIter cff_ccwiter(FaceHandle _fh) const;
  
  // 'begin' circulators
  
  /// vertex - vertex circulator
  VertexVertexIter vv_begin(VertexHandle _vh);
  /// vertex - vertex circulator cw
  VertexVertexCWIter vv_cwbegin(VertexHandle _vh);
  /// vertex - vertex circulator ccw
  VertexVertexCCWIter vv_ccwbegin(VertexHandle _vh);
  /// vertex - incoming halfedge circulator
  VertexIHalfedgeIter vih_begin(VertexHandle _vh);
  /// vertex - incoming halfedge circulator cw
  VertexIHalfedgeCWIter vih_cwbegin(VertexHandle _vh);
  /// vertex - incoming halfedge circulator ccw
  VertexIHalfedgeCCWIter vih_ccwbegin(VertexHandle _vh);
  /// vertex - outgoing halfedge circulator
  VertexOHalfedgeIter voh_begin(VertexHandle _vh);
  /// vertex - outgoing halfedge circulator cw
  VertexOHalfedgeCWIter voh_cwbegin(VertexHandle _vh);
  /// vertex - outgoing halfedge circulator ccw
  VertexOHalfedgeCCWIter voh_ccwbegin(VertexHandle _vh);
  /// vertex - edge circulator
  VertexEdgeIter ve_begin(VertexHandle _vh);
  /// vertex - edge circulator cw
  VertexEdgeCWIter ve_cwbegin(VertexHandle _vh);
  /// vertex - edge circulator ccw
  VertexEdgeCCWIter ve_ccwbegin(VertexHandle _vh);
  /// vertex - face circulator
  VertexFaceIter vf_begin(VertexHandle _vh);
  /// vertex - face circulator cw
  VertexFaceCWIter vf_cwbegin(VertexHandle _vh);
  /// vertex - face circulator ccw
  VertexFaceCCWIter vf_ccwbegin(VertexHandle _vh);


  /// const vertex circulator
  ConstVertexVertexIter cvv_begin(VertexHandle _vh) const;
  /// const vertex circulator cw
  ConstVertexVertexCWIter cvv_cwbegin(VertexHandle _vh) const;
  /// const vertex circulator ccw
  ConstVertexVertexCCWIter cvv_ccwbegin(VertexHandle _vh) const;
  /// const vertex - incoming halfedge circulator
  ConstVertexIHalfedgeIter cvih_begin(VertexHandle _vh) const;
  /// const vertex - incoming halfedge circulator cw
  ConstVertexIHalfedgeCWIter cvih_cwbegin(VertexHandle _vh) const;
  /// const vertex - incoming halfedge circulator ccw
  ConstVertexIHalfedgeCCWIter cvih_ccwbegin(VertexHandle _vh) const;
  /// const vertex - outgoing halfedge circulator
  ConstVertexOHalfedgeIter cvoh_begin(VertexHandle _vh) const;
  /// const vertex - outgoing halfedge circulator cw
  ConstVertexOHalfedgeCWIter cvoh_cwbegin(VertexHandle _vh) const;
  /// const vertex - outgoing halfedge circulator ccw
  ConstVertexOHalfedgeCCWIter cvoh_ccwbegin(VertexHandle _vh) const;
  /// const vertex - edge circulator
  ConstVertexEdgeIter cve_begin(VertexHandle _vh) const;
  /// const vertex - edge circulator cw
  ConstVertexEdgeCWIter cve_cwbegin(VertexHandle _vh) const;
  /// const vertex - edge circulator ccw
  ConstVertexEdgeCCWIter cve_ccwbegin(VertexHandle _vh) const;
  /// const vertex - face circulator
  ConstVertexFaceIter cvf_begin(VertexHandle _vh) const;
  /// const vertex - face circulator cw
  ConstVertexFaceCWIter cvf_cwbegin(VertexHandle _vh) const;
  /// const vertex - face circulator ccw
  ConstVertexFaceCCWIter cvf_ccwbegin(VertexHandle _vh) const;

  /// face - vertex circulator
  FaceVertexIter fv_begin(FaceHandle _fh);
  /// face - vertex circulator cw
  FaceVertexCWIter fv_cwbegin(FaceHandle _fh);
  /// face - vertex circulator ccw
  FaceVertexCCWIter fv_ccwbegin(FaceHandle _fh);
  /// face - halfedge circulator
  FaceHalfedgeIter fh_begin(FaceHandle _fh);
  /// face - halfedge circulator cw
  FaceHalfedgeCWIter fh_cwbegin(FaceHandle _fh);
  /// face - halfedge circulator ccw
  FaceHalfedgeCCWIter fh_ccwbegin(FaceHandle _fh);
  /// face - edge circulator
  FaceEdgeIter fe_begin(FaceHandle _fh);
  /// face - edge circulator cw
  FaceEdgeCWIter fe_cwbegin(FaceHandle _fh);
  /// face - edge circulator ccw
  FaceEdgeCCWIter fe_ccwbegin(FaceHandle _fh);
  /// face - face circulator
  FaceFaceIter ff_begin(FaceHandle _fh);
  /// face - face circulator cw
  FaceFaceCWIter ff_cwbegin(FaceHandle _fh);
  /// face - face circulator ccw
  FaceFaceCCWIter ff_ccwbegin(FaceHandle _fh);
  /// halfedge circulator
  HalfedgeLoopIter hl_begin(HalfedgeHandle _heh);
  /// halfedge circulator
  HalfedgeLoopCWIter hl_cwbegin(HalfedgeHandle _heh);
  /// halfedge circulator ccw
  HalfedgeLoopCCWIter hl_ccwbegin(HalfedgeHandle _heh);

  /// const face - vertex circulator
  ConstFaceVertexIter cfv_begin(FaceHandle _fh) const;
  /// const face - vertex circulator cw
  ConstFaceVertexCWIter cfv_cwbegin(FaceHandle _fh) const;
  /// const face - vertex circulator ccw
  ConstFaceVertexCCWIter cfv_ccwbegin(FaceHandle _fh) const;
  /// const face - halfedge circulator
  ConstFaceHalfedgeIter cfh_begin(FaceHandle _fh) const;
  /// const face - halfedge circulator cw
  ConstFaceHalfedgeCWIter cfh_cwbegin(FaceHandle _fh) const;
  /// const face - halfedge circulator ccw
  ConstFaceHalfedgeCCWIter cfh_ccwbegin(FaceHandle _fh) const;
  /// const face - edge circulator
  ConstFaceEdgeIter cfe_begin(FaceHandle _fh) const;
  /// const face - edge circulator cw
  ConstFaceEdgeCWIter cfe_cwbegin(FaceHandle _fh) const;
  /// const face - edge circulator ccw
  ConstFaceEdgeCCWIter cfe_ccwbegin(FaceHandle _fh) const;
  /// const face - face circulator
  ConstFaceFaceIter cff_begin(FaceHandle _fh) const;
  /// const face - face circulator cw
  ConstFaceFaceCWIter cff_cwbegin(FaceHandle _fh) const;
  /// const face - face circulator ccw
  ConstFaceFaceCCWIter cff_ccwbegin(FaceHandle _fh) const;
  /// const halfedge circulator
  ConstHalfedgeLoopIter chl_begin(HalfedgeHandle _heh) const;
  /// const halfedge circulator cw
  ConstHalfedgeLoopCWIter chl_cwbegin(HalfedgeHandle _heh) const;
  /// const halfedge circulator ccw
  ConstHalfedgeLoopCCWIter chl_ccwbegin(HalfedgeHandle _heh) const;
  
  // 'end' circulators
  
  /// vertex - vertex circulator
  VertexVertexIter vv_end(VertexHandle _vh);
  /// vertex - vertex circulator cw
  VertexVertexCWIter vv_cwend(VertexHandle _vh);
  /// vertex - vertex circulator ccw
  VertexVertexCCWIter vv_ccwend(VertexHandle _vh);
  /// vertex - incoming halfedge circulator
  VertexIHalfedgeIter vih_end(VertexHandle _vh);
  /// vertex - incoming halfedge circulator cw
  VertexIHalfedgeCWIter vih_cwend(VertexHandle _vh);
  /// vertex - incoming halfedge circulator ccw
  VertexIHalfedgeCCWIter vih_ccwend(VertexHandle _vh);
  /// vertex - outgoing halfedge circulator
  VertexOHalfedgeIter voh_end(VertexHandle _vh);
  /// vertex - outgoing halfedge circulator cw
  VertexOHalfedgeCWIter voh_cwend(VertexHandle _vh);
  /// vertex - outgoing halfedge circulator ccw
  VertexOHalfedgeCCWIter voh_ccwend(VertexHandle _vh);
  /// vertex - edge circulator
  VertexEdgeIter ve_end(VertexHandle _vh);
  /// vertex - edge circulator cw
  VertexEdgeCWIter ve_cwend(VertexHandle _vh);
  /// vertex - edge circulator ccw
  VertexEdgeCCWIter ve_ccwend(VertexHandle _vh);
  /// vertex - face circulator
  VertexFaceIter vf_end(VertexHandle _vh);
  /// vertex - face circulator cw
  VertexFaceCWIter vf_cwend(VertexHandle _vh);
  /// vertex - face circulator ccw
  VertexFaceCCWIter vf_ccwend(VertexHandle _vh);

  /// const vertex circulator
  ConstVertexVertexIter cvv_end(VertexHandle _vh) const;
  /// const vertex circulator cw
  ConstVertexVertexCWIter cvv_cwend(VertexHandle _vh) const;
  /// const vertex circulator ccw
  ConstVertexVertexCCWIter cvv_ccwend(VertexHandle _vh) const;
  /// const vertex - incoming halfedge circulator
  ConstVertexIHalfedgeIter cvih_end(VertexHandle _vh) const;
  /// const vertex - incoming halfedge circulator cw
  ConstVertexIHalfedgeCWIter cvih_cwend(VertexHandle _vh) const;
  /// const vertex - incoming halfedge circulator ccw
  ConstVertexIHalfedgeCCWIter cvih_ccwend(VertexHandle _vh) const;
  /// const vertex - outgoing halfedge circulator
  ConstVertexOHalfedgeIter cvoh_end(VertexHandle _vh) const;
  /// const vertex - outgoing halfedge circulator cw
  ConstVertexOHalfedgeCWIter cvoh_cwend(VertexHandle _vh) const;
  /// const vertex - outgoing halfedge circulator ccw
  ConstVertexOHalfedgeCCWIter cvoh_ccwend(VertexHandle _vh) const;
  /// const vertex - edge circulator
  ConstVertexEdgeIter cve_end(VertexHandle _vh) const;
  /// const vertex - edge circulator cw
  ConstVertexEdgeCWIter cve_cwend(VertexHandle _vh) const;
  /// const vertex - edge circulator ccw
  ConstVertexEdgeCCWIter cve_ccwend(VertexHandle _vh) const;
  /// const vertex - face circulator
  ConstVertexFaceIter cvf_end(VertexHandle _vh) const;
  /// const vertex - face circulator cw
  ConstVertexFaceCWIter cvf_cwend(VertexHandle _vh) const;
  /// const vertex - face circulator ccw
  ConstVertexFaceCCWIter cvf_ccwend(VertexHandle _vh) const;

  /// face - vertex circulator
  FaceVertexIter fv_end(FaceHandle _fh);
  /// face - vertex circulator cw
  FaceVertexCWIter fv_cwend(FaceHandle _fh);
  /// face - vertex circulator ccw
  FaceVertexCCWIter fv_ccwend(FaceHandle _fh);
  /// face - halfedge circulator
  FaceHalfedgeIter fh_end(FaceHandle _fh);
  /// face - halfedge circulator cw
  FaceHalfedgeCWIter fh_cwend(FaceHandle _fh);
  /// face - halfedge circulator ccw
  FaceHalfedgeCCWIter fh_ccwend(FaceHandle _fh);
  /// face - edge circulator
  FaceEdgeIter fe_end(FaceHandle _fh);
  /// face - edge circulator cw
  FaceEdgeCWIter fe_cwend(FaceHandle _fh);
  /// face - edge circulator ccw
  FaceEdgeCCWIter fe_ccwend(FaceHandle _fh);
  /// face - face circulator
  FaceFaceIter ff_end(FaceHandle _fh);
  /// face - face circulator cw
  FaceFaceCWIter ff_cwend(FaceHandle _fh);
  /// face - face circulator ccw
  FaceFaceCCWIter ff_ccwend(FaceHandle _fh);
  /// face - face circulator
  HalfedgeLoopIter hl_end(HalfedgeHandle _heh);
  /// face - face circulator cw
  HalfedgeLoopCWIter hl_cwend(HalfedgeHandle _heh);
  /// face - face circulator ccw
  HalfedgeLoopCCWIter hl_ccwend(HalfedgeHandle _heh);

  /// const face - vertex circulator
  ConstFaceVertexIter cfv_end(FaceHandle _fh) const;
  /// const face - vertex circulator cw
  ConstFaceVertexCWIter cfv_cwend(FaceHandle _fh) const;
  /// const face - vertex circulator ccw
  ConstFaceVertexCCWIter cfv_ccwend(FaceHandle _fh) const;
  /// const face - halfedge circulator
  ConstFaceHalfedgeIter cfh_end(FaceHandle _fh) const;
  /// const face - halfedge circulator cw
  ConstFaceHalfedgeCWIter cfh_cwend(FaceHandle _fh) const;
  /// const face - halfedge circulator ccw
  ConstFaceHalfedgeCCWIter cfh_ccwend(FaceHandle _fh) const;
  /// const face - edge circulator
  ConstFaceEdgeIter cfe_end(FaceHandle _fh) const;
  /// const face - edge circulator cw
  ConstFaceEdgeCWIter cfe_cwend(FaceHandle _fh) const;
  /// const face - edge circulator ccw
  ConstFaceEdgeCCWIter cfe_ccwend(FaceHandle _fh) const;
  /// const face - face circulator
  ConstFaceFaceIter cff_end(FaceHandle _fh) const;
  /// const face - face circulator
  ConstFaceFaceCWIter cff_cwend(FaceHandle _fh) const;
  /// const face - face circulator
  ConstFaceFaceCCWIter cff_ccwend(FaceHandle _fh) const;
  /// const face - face circulator
  ConstHalfedgeLoopIter chl_end(HalfedgeHandle _heh) const;
  /// const face - face circulator cw
  ConstHalfedgeLoopCWIter chl_cwend(HalfedgeHandle _heh) const;
  /// const face - face circulator ccw
  ConstHalfedgeLoopCCWIter chl_ccwend(HalfedgeHandle _heh) const;
  //@}

  /** @name Range based iterators and circulators */
  //@{

  typedef EntityRange<RangeTraitT<
          const PolyConnectivity,
          PolyConnectivity::ConstVertexIter,
          &PolyConnectivity::vertices_begin,
          &PolyConnectivity::vertices_end>> ConstVertexRange;
  typedef EntityRange<RangeTraitT<
          const PolyConnectivity,
          PolyConnectivity::ConstVertexIter,
          &PolyConnectivity::vertices_sbegin,
          &PolyConnectivity::vertices_end>> ConstVertexRangeSkipping;
  typedef EntityRange<RangeTraitT<
          const PolyConnectivity,
          PolyConnectivity::ConstHalfedgeIter,
          &PolyConnectivity::halfedges_begin,
          &PolyConnectivity::halfedges_end>> ConstHalfedgeRange;
  typedef EntityRange<RangeTraitT<
          const PolyConnectivity,
          PolyConnectivity::ConstHalfedgeIter,
          &PolyConnectivity::halfedges_sbegin,
          &PolyConnectivity::halfedges_end>> ConstHalfedgeRangeSkipping;
  typedef EntityRange<RangeTraitT<
          const PolyConnectivity,
          PolyConnectivity::ConstEdgeIter,
          &PolyConnectivity::edges_begin,
          &PolyConnectivity::edges_end>> ConstEdgeRange;
  typedef EntityRange<RangeTraitT<
          const PolyConnectivity,
          PolyConnectivity::ConstEdgeIter,
          &PolyConnectivity::edges_sbegin,
          &PolyConnectivity::edges_end>> ConstEdgeRangeSkipping;
  typedef EntityRange<RangeTraitT<
          const PolyConnectivity,
          PolyConnectivity::ConstFaceIter,
          &PolyConnectivity::faces_begin,
          &PolyConnectivity::faces_end>> ConstFaceRange;
  typedef EntityRange<RangeTraitT<
          const PolyConnectivity,
          PolyConnectivity::ConstFaceIter,
          &PolyConnectivity::faces_sbegin,
          &PolyConnectivity::faces_end>> ConstFaceRangeSkipping;


  /**
   * @return The vertices as a range object suitable
   * for C++11 range based for loops. Will skip deleted vertices.
   */
  ConstVertexRangeSkipping vertices() const;

  /**
   * @return The vertices as a range object suitable
   * for C++11 range based for loops. Will include deleted vertices.
   */
  ConstVertexRange all_vertices() const;

  /**
   * @return The halfedges as a range object suitable
   * for C++11 range based for loops. Will skip deleted halfedges.
   */
  ConstHalfedgeRangeSkipping halfedges() const;

  /**
   * @return The halfedges as a range object suitable
   * for C++11 range based for loops. Will include deleted halfedges.
   */
  ConstHalfedgeRange all_halfedges() const;

  /**
   * @return The edges as a range object suitable
   * for C++11 range based for loops. Will skip deleted edges.
   */
  ConstEdgeRangeSkipping edges() const;

  /**
   * @return The edges as a range object suitable
   * for C++11 range based for loops. Will include deleted edges.
   */
  ConstEdgeRange all_edges() const;

  /**
   * @return The faces as a range object suitable
   * for C++11 range based for loops. Will skip deleted faces.
   */
  ConstFaceRangeSkipping faces() const;

  /**
   * @return The faces as a range object suitable
   * for C++11 range based for loops. Will include deleted faces.
   */
  ConstFaceRange all_faces() const;


  typedef CirculatorRange<CirculatorRangeTraitT<
          PolyConnectivity,
          ConstVertexVertexCWIter,
  VertexHandle,
  VertexHandle,
          &PolyConnectivity::cvv_cwbegin,
          &PolyConnectivity::cvv_cwend>> ConstVertexVertexRange;
  typedef CirculatorRange<CirculatorRangeTraitT<
          PolyConnectivity,
          ConstVertexIHalfedgeIter,
          VertexHandle,
          HalfedgeHandle,
          &PolyConnectivity::cvih_begin,
          &PolyConnectivity::cvih_end>> ConstVertexIHalfedgeRange;
  typedef CirculatorRange<CirculatorRangeTraitT<
          PolyConnectivity,
          ConstVertexOHalfedgeIter,
          VertexHandle,
          HalfedgeHandle,
          &PolyConnectivity::cvoh_begin,
          &PolyConnectivity::cvoh_end>> ConstVertexOHalfedgeRange;
  typedef CirculatorRange<CirculatorRangeTraitT<
          PolyConnectivity,
          ConstVertexEdgeIter,
          VertexHandle,
          EdgeHandle,
          &PolyConnectivity::cve_begin,
          &PolyConnectivity::cve_end>> ConstVertexEdgeRange;
  typedef CirculatorRange<CirculatorRangeTraitT<
          PolyConnectivity,
          ConstVertexFaceIter,
          VertexHandle,
          FaceHandle,
          &PolyConnectivity::cvf_begin,
          &PolyConnectivity::cvf_end>> ConstVertexFaceRange;
  typedef CirculatorRange<CirculatorRangeTraitT<
          PolyConnectivity,
          ConstFaceVertexIter,
          FaceHandle,
          VertexHandle,
          &PolyConnectivity::cfv_begin,
          &PolyConnectivity::cfv_end>> ConstFaceVertexRange;
  typedef CirculatorRange<CirculatorRangeTraitT<
          PolyConnectivity,
          ConstFaceHalfedgeIter,
          FaceHandle,
          HalfedgeHandle,
          &PolyConnectivity::cfh_begin,
          &PolyConnectivity::cfh_end>> ConstFaceHalfedgeRange;
  typedef CirculatorRange<CirculatorRangeTraitT<
          PolyConnectivity,
          ConstFaceEdgeIter,
          FaceHandle,
          EdgeHandle,
          &PolyConnectivity::cfe_begin,
          &PolyConnectivity::cfe_end>> ConstFaceEdgeRange;
  typedef CirculatorRange<CirculatorRangeTraitT<
          PolyConnectivity,
          ConstFaceFaceIter,
          FaceHandle,
          FaceHandle,
          &PolyConnectivity::cff_begin,
          &PolyConnectivity::cff_end>> ConstFaceFaceRange;

  /**
   * @return The vertices adjacent to the specified vertex
   * as a range object suitable for C++11 range based for loops.
   */
  ConstVertexVertexRange vv_range(VertexHandle _vh) const;

  /**
   * @return The incoming halfedges incident to the specified vertex
   * as a range object suitable for C++11 range based for loops.
   */
  ConstVertexIHalfedgeRange vih_range(VertexHandle _vh) const;

  /**
   * @return The outgoing halfedges incident to the specified vertex
   * as a range object suitable for C++11 range based for loops.
   */
  ConstVertexOHalfedgeRange voh_range(VertexHandle _vh) const;

  /**
   * @return The edges incident to the specified vertex
   * as a range object suitable for C++11 range based for loops.
   */
  ConstVertexEdgeRange ve_range(VertexHandle _vh) const ;

  /**
   * @return The faces incident to the specified vertex
   * as a range object suitable for C++11 range based for loops.
   */
  ConstVertexFaceRange vf_range(VertexHandle _vh) const;

  /**
   * @return The vertices incident to the specified face
   * as a range object suitable for C++11 range based for loops.
   */
  ConstFaceVertexRange fv_range(FaceHandle _fh) const;

  /**
   * @return The halfedges incident to the specified face
   * as a range object suitable for C++11 range based for loops.
   */
  ConstFaceHalfedgeRange fh_range(FaceHandle _fh) const;

  /**
   * @return The edges incident to the specified face
   * as a range object suitable for C++11 range based for loops.
   */
  ConstFaceEdgeRange fe_range(FaceHandle _fh) const;

  /**
   * @return The faces adjacent to the specified face
   * as a range object suitable for C++11 range based for loops.
   */
  ConstFaceFaceRange ff_range(FaceHandle _fh) const;

  //@}

  //===========================================================================
  /** @name Boundary and manifold tests
   * @{ */
  //===========================================================================

  /** \brief Check if the halfedge is at the boundary
   *
   * The halfedge is at the boundary, if no face is incident to it.
   *
   * @param _heh HalfedgeHandle to test
   * @return boundary?
   */
  bool is_boundary(HalfedgeHandle _heh) const
  { return ArrayKernel::is_boundary(_heh); }

  /** \brief Is the edge a boundary edge?
   *
   * Checks it the edge _eh is a boundary edge, i.e. is one of its halfedges
   * a boundary halfedge.
   *
   * @param _eh Edge handle to test
   * @return boundary?
   */
  bool is_boundary(EdgeHandle _eh) const
  {
    return (is_boundary(halfedge_handle(_eh, 0)) ||
            is_boundary(halfedge_handle(_eh, 1)));
  }

  /** \brief Is vertex _vh a boundary vertex ?
   *
   * Checks if the associated halfedge (which would on a boundary be the outside
   * halfedge), is connected to a face. Which is equivalent, if the vertex is
   * at the boundary of the mesh, as OpenMesh will make sure, that if there is a
   * boundary halfedge at the vertex, the halfedge will be the one which is associated
   * to the vertex.
   *
   * @param _vh VertexHandle to test
   * @return boundary?
   */
  bool is_boundary(VertexHandle _vh) const
  {
    HalfedgeHandle heh(halfedge_handle(_vh));
    return (!(heh.is_valid() && face_handle(heh).is_valid()));
  }

  /** \brief Check if face is at the boundary
   *
   * Is face _fh at boundary, i.e. is one of its edges (or vertices)
   * a boundary edge?
   *
   * @param _fh Check this face
   * @param _check_vertex If \c true, check the corner vertices of the face, too.
   * @return boundary?
   */
  bool is_boundary(FaceHandle _fh, bool _check_vertex=false) const;

  /** \brief Is (the mesh at) vertex _vh  two-manifold ?
   *
   * The vertex is non-manifold if more than one gap exists, i.e.
   * more than one outgoing boundary halfedge. If (at least) one
   * boundary halfedge exists, the vertex' halfedge must be a
   * boundary halfedge.
   *
   * @param _vh VertexHandle to test
   * @return manifold?
   */
  bool is_manifold(VertexHandle _vh) const;

  /** @} */

  // --- shortcuts ---
  
  /// returns the face handle of the opposite halfedge 
  inline FaceHandle opposite_face_handle(HalfedgeHandle _heh) const
  { return face_handle(opposite_halfedge_handle(_heh)); }
    
  // --- misc ---

  /** Adjust outgoing halfedge handle for vertices, so that it is a
      boundary halfedge whenever possible. 
  */
  void adjust_outgoing_halfedge(VertexHandle _vh);

  /// Find halfedge from _vh0 to _vh1. Returns invalid handle if not found.
  HalfedgeHandle find_halfedge(VertexHandle _start_vh, VertexHandle _end_vh) const;
  /// Vertex valence
  uint valence(VertexHandle _vh) const;
  /// Face valence
  uint valence(FaceHandle _fh) const;
  
  // --- connectivity operattions 
    
  /** Halfedge collapse: collapse the from-vertex of halfedge _heh
      into its to-vertex.

      \attention Needs vertex/edge/face status attribute in order to
      delete the items that degenerate.

      \note The from vertex is marked as deleted while the to vertex will still exist.

      \note This function does not perform a garbage collection. It
      only marks degenerate items as deleted.

      \attention A halfedge collapse may lead to topological inconsistencies.
      Therefore you should check this using is_collapse_ok().  
  */
  void collapse(HalfedgeHandle _heh);
  /** return true if the this the only link between the faces adjacent to _eh.
      _eh is allowed to be boundary, in which case true is returned iff _eh is 
      the only boundary edge of its ajdacent face.
  */
  bool is_simple_link(EdgeHandle _eh) const;
  /** return true if _fh shares only one edge with all of its adjacent faces.
      Boundary is treated as one face, i.e., the function false if _fh has more
      than one boundary edge.
  */
  bool is_simply_connected(FaceHandle _fh) const;
  /** Removes the edge _eh. Its adjacent faces are merged. _eh and one of the 
      adjacent faces are set deleted. The handle of the remaining face is 
      returned (InvalidFaceHandle is returned if _eh is a boundary edge).
      
      \pre is_simple_link(_eh). This ensures that there are no hole faces
      or isolated vertices appearing in result of the operation.
      
      \attention Needs the Attributes::Status attribute for edges and faces.
      
      \note This function does not perform a garbage collection. It
      only marks items as deleted.
  */
  FaceHandle remove_edge(EdgeHandle _eh);
  /** Inverse of remove_edge. _eh should be the handle of the edge and the
      vertex and halfedge handles pointed by edge(_eh) should be valid. 
  */
  void reinsert_edge(EdgeHandle _eh);
  /** Inserts an edge between to_vh(_prev_heh) and from_vh(_next_heh).
      A new face is created started at heh0 of the inserted edge and
      its halfedges loop includes both _prev_heh and _next_heh. If an 
      old face existed which includes the argument halfedges, it is 
      split at the new edge. heh0 is returned. 
      
      \note assumes _prev_heh and _next_heh are either boundary or pointed
      to the same face
  */
  HalfedgeHandle insert_edge(HalfedgeHandle _prev_heh, HalfedgeHandle _next_heh);
    
  /** \brief Face split (= 1-to-n split).
     *
     * Split an arbitrary face into triangles by connecting each vertex of fh to vh.
     *
     * \note fh will remain valid (it will become one of the triangles)
     * \note the halfedge handles of the new triangles will point to the old halfeges
     *
     * \note The properties of the new faces and all other new primitives will be undefined!
     *
     * @param _fh Face handle that should be splitted
     * @param _vh Vertex handle of the new vertex that will be inserted in the face
     */
  void split(FaceHandle _fh, VertexHandle _vh);

  /** \brief Face split (= 1-to-n split).
   *
   * Split an arbitrary face into triangles by connecting each vertex of fh to vh.
   *
   * \note fh will remain valid (it will become one of the triangles)
   * \note the halfedge handles of the new triangles will point to the old halfeges
   *
   * \note The properties of the new faces will be adjusted to the properties of the original faces
   * \note Properties of the new edges and halfedges will be undefined
   *
   * @param _fh Face handle that should be splitted
   * @param _vh Vertex handle of the new vertex that will be inserted in the face
   */
  void split_copy(FaceHandle _fh, VertexHandle _vh);
  
  /** \brief Triangulate the face _fh

    Split an arbitrary face into triangles by connecting
    each vertex of fh after its second to vh.

    \note _fh will remain valid (it will become one of the
      triangles)

    \note The halfedge handles of the new triangles will
      point to the old halfedges

    @param _fh Handle of the face that should be triangulated
  */
  void triangulate(FaceHandle _fh);

  /** \brief triangulate the entire mesh  
  */
  void triangulate();
  
  /** Edge split (inserts a vertex on the edge only)
   *
   * This edge split only splits the edge without introducing new faces!
   * As this is for polygonal meshes, we can have valence 2 vertices here.
   *
   * \note The properties of the new edges and halfedges will be undefined!
   *
   * @param _eh Handle of the edge, that will be splitted
   * @param _vh Handle of the vertex that will be inserted at the edge
   */
  void split_edge(EdgeHandle _eh, VertexHandle _vh);

  /** Edge split (inserts a vertex on the edge only)
   *
   * This edge split only splits the edge without introducing new faces!
   * As this is for polygonal meshes, we can have valence 2 vertices here.
   *
   * \note The properties of the new edge will be copied from the splitted edge
   * \note Properties of the new halfedges will be undefined
   *
   * @param _eh Handle of the edge, that will be splitted
   * @param _vh Handle of the vertex that will be inserted at the edge
   */
  void split_edge_copy(EdgeHandle _eh, VertexHandle _vh);


  /** \name Generic handle derefertiation.
      Calls the respective vertex(), halfedge(), edge(), face()
      method of the mesh kernel.
  */
  //@{
  /// Get item from handle
  const Vertex&    deref(VertexHandle _h)   const { return vertex(_h); }
  Vertex&          deref(VertexHandle _h)         { return vertex(_h); }
  const Halfedge&  deref(HalfedgeHandle _h) const { return halfedge(_h); }
  Halfedge&        deref(HalfedgeHandle _h)       { return halfedge(_h); }
  const Edge&      deref(EdgeHandle _h)     const { return edge(_h); }
  Edge&            deref(EdgeHandle _h)           { return edge(_h); }
  const Face&      deref(FaceHandle _h)     const { return face(_h); }
  Face&            deref(FaceHandle _h)           { return face(_h); }
  //@}

protected:  
  /// Helper for halfedge collapse
  void collapse_edge(HalfedgeHandle _hh);
  /// Helper for halfedge collapse
  void collapse_loop(HalfedgeHandle _hh);



private: // Working storage for add_face()
       struct AddFaceEdgeInfo
       {
               HalfedgeHandle halfedge_handle;
               bool is_new;
               bool needs_adjust;
       };
       std::vector<AddFaceEdgeInfo> edgeData_; //
       std::vector<std::pair<HalfedgeHandle, HalfedgeHandle> > next_cache_; // cache for set_next_halfedge and vertex' set_halfedge

};

}//namespace OpenMesh


#include <OpenMesh/Core/Mesh/IteratorsT.hh>
#include <OpenMesh/Core/Mesh/CirculatorsT.hh>

namespace OpenMesh {

/// Generic class for vertex/halfedge/edge/face ranges.
template <typename RangeTraitT>
class EntityRange : public SmartRangeT<EntityRange<RangeTraitT>, typename RangeTraitT::ITER_TYPE::SmartHandle> {
    public:
        typedef typename RangeTraitT::ITER_TYPE iterator;
        typedef typename RangeTraitT::ITER_TYPE const_iterator;

        explicit EntityRange(typename RangeTraitT::CONTAINER_TYPE &container) : container_(container) {}
        typename RangeTraitT::ITER_TYPE begin() const { return RangeTraitT::begin(container_); }
        typename RangeTraitT::ITER_TYPE end() const { return RangeTraitT::end(container_); }

    private:
        typename RangeTraitT::CONTAINER_TYPE &container_;
};

/// Generic class for iterator ranges.
template <typename CirculatorRangeTraitT>
//class CirculatorRange : public SmartRangeT<CirculatorRange<CirculatorRangeTraitT>, decltype (make_smart(std::declval<typename CirculatorRangeTraitT::TO_ENTITYE_TYPE>(), std::declval<PolyConnectivity>()))>{
class CirculatorRange : public SmartRangeT<CirculatorRange<CirculatorRangeTraitT>, typename SmartHandle<typename CirculatorRangeTraitT::TO_ENTITYE_TYPE>::type>{
    public:
        typedef typename CirculatorRangeTraitT::ITER_TYPE ITER_TYPE;
        typedef typename CirculatorRangeTraitT::CENTER_ENTITY_TYPE CENTER_ENTITY_TYPE;
        typedef typename CirculatorRangeTraitT::CONTAINER_TYPE CONTAINER_TYPE;
        typedef ITER_TYPE iterator;
        typedef ITER_TYPE const_iterator;

        CirculatorRange(
                const CONTAINER_TYPE &container,
                CENTER_ENTITY_TYPE center) :
            container_(container), center_(center) {}
        ITER_TYPE begin() const { return CirculatorRangeTraitT::begin(container_, center_); }
        ITER_TYPE end() const { return CirculatorRangeTraitT::end(container_, center_); }

    private:
        const CONTAINER_TYPE &container_;
        CENTER_ENTITY_TYPE center_;
};


inline PolyConnectivity::ConstVertexRangeSkipping   PolyConnectivity::vertices()      const { return ConstVertexRangeSkipping(*this);   }
inline PolyConnectivity::ConstVertexRange           PolyConnectivity::all_vertices()  const { return ConstVertexRange(*this);           }
inline PolyConnectivity::ConstHalfedgeRangeSkipping PolyConnectivity::halfedges()     const { return ConstHalfedgeRangeSkipping(*this); }
inline PolyConnectivity::ConstHalfedgeRange         PolyConnectivity::all_halfedges() const { return ConstHalfedgeRange(*this);         }
inline PolyConnectivity::ConstEdgeRangeSkipping     PolyConnectivity::edges()         const { return ConstEdgeRangeSkipping(*this);     }
inline PolyConnectivity::ConstEdgeRange             PolyConnectivity::all_edges()     const { return ConstEdgeRange(*this);             }
inline PolyConnectivity::ConstFaceRangeSkipping     PolyConnectivity::faces()         const { return ConstFaceRangeSkipping(*this);     }
inline PolyConnectivity::ConstFaceRange             PolyConnectivity::all_faces()     const { return ConstFaceRange(*this);             }

inline PolyConnectivity::ConstVertexVertexRange PolyConnectivity::vv_range(VertexHandle _vh) const {
    return ConstVertexVertexRange(*this, _vh);
}

inline PolyConnectivity::ConstVertexIHalfedgeRange PolyConnectivity::vih_range(VertexHandle _vh) const {
    return ConstVertexIHalfedgeRange(*this, _vh);
}

inline PolyConnectivity::ConstVertexOHalfedgeRange PolyConnectivity::voh_range(VertexHandle _vh) const {
    return ConstVertexOHalfedgeRange(*this, _vh);
}

inline PolyConnectivity::ConstVertexEdgeRange PolyConnectivity::ve_range(VertexHandle _vh) const {
    return ConstVertexEdgeRange(*this, _vh);
}

inline PolyConnectivity::ConstVertexFaceRange PolyConnectivity::vf_range(VertexHandle _vh) const {
    return ConstVertexFaceRange(*this, _vh);
}

inline PolyConnectivity::ConstFaceVertexRange PolyConnectivity::fv_range(FaceHandle _fh) const {
    return ConstFaceVertexRange(*this, _fh);
}

inline PolyConnectivity::ConstFaceHalfedgeRange PolyConnectivity::fh_range(FaceHandle _fh) const {
    return ConstFaceHalfedgeRange(*this, _fh);
}

inline PolyConnectivity::ConstFaceEdgeRange PolyConnectivity::fe_range(FaceHandle _fh) const {
    return ConstFaceEdgeRange(*this, _fh);
}

inline PolyConnectivity::ConstFaceFaceRange PolyConnectivity::ff_range(FaceHandle _fh) const {
    return ConstFaceFaceRange(*this, _fh);
}



inline PolyConnectivity::VertexIter PolyConnectivity::vertices_begin()
{  return VertexIter(*this, VertexHandle(0)); }

inline PolyConnectivity::ConstVertexIter PolyConnectivity::vertices_begin() const
{  return ConstVertexIter(*this, VertexHandle(0)); }

inline PolyConnectivity::VertexIter PolyConnectivity::vertices_end()
{  return VertexIter(*this, VertexHandle( int(n_vertices() ) )); }

inline PolyConnectivity::ConstVertexIter PolyConnectivity::vertices_end() const
{  return ConstVertexIter(*this, VertexHandle( int(n_vertices()) )); }

inline PolyConnectivity::HalfedgeIter PolyConnectivity::halfedges_begin()
{  return HalfedgeIter(*this, HalfedgeHandle(0)); }

inline PolyConnectivity::ConstHalfedgeIter PolyConnectivity::halfedges_begin() const
{  return ConstHalfedgeIter(*this, HalfedgeHandle(0)); }

inline PolyConnectivity::HalfedgeIter PolyConnectivity::halfedges_end()
{  return HalfedgeIter(*this, HalfedgeHandle(int(n_halfedges()))); }

inline PolyConnectivity::ConstHalfedgeIter PolyConnectivity::halfedges_end() const
{  return ConstHalfedgeIter(*this, HalfedgeHandle(int(n_halfedges()))); }

inline PolyConnectivity::EdgeIter PolyConnectivity::edges_begin()
{  return EdgeIter(*this, EdgeHandle(0)); }

inline PolyConnectivity::ConstEdgeIter PolyConnectivity::edges_begin() const
{  return ConstEdgeIter(*this, EdgeHandle(0)); }

inline PolyConnectivity::EdgeIter PolyConnectivity::edges_end()
{  return EdgeIter(*this, EdgeHandle(int(n_edges()))); }

inline PolyConnectivity::ConstEdgeIter PolyConnectivity::edges_end() const
{  return ConstEdgeIter(*this, EdgeHandle(int(n_edges()))); }

inline PolyConnectivity::FaceIter PolyConnectivity::faces_begin()
{  return FaceIter(*this, FaceHandle(0)); }

inline PolyConnectivity::ConstFaceIter PolyConnectivity::faces_begin() const
{  return ConstFaceIter(*this, FaceHandle(0)); }

inline PolyConnectivity::FaceIter PolyConnectivity::faces_end()
{  return FaceIter(*this, FaceHandle(int(n_faces()))); }


inline PolyConnectivity::ConstFaceIter PolyConnectivity::faces_end() const
{  return ConstFaceIter(*this, FaceHandle(int(n_faces()))); }

inline PolyConnectivity::VertexIter PolyConnectivity::vertices_sbegin()
{  return VertexIter(*this, VertexHandle(0), true); }

inline PolyConnectivity::ConstVertexIter PolyConnectivity::vertices_sbegin() const
{  return ConstVertexIter(*this, VertexHandle(0), true); }

inline PolyConnectivity::HalfedgeIter PolyConnectivity::halfedges_sbegin()
{  return HalfedgeIter(*this, HalfedgeHandle(0), true); }

inline PolyConnectivity::ConstHalfedgeIter PolyConnectivity::halfedges_sbegin() const
{  return ConstHalfedgeIter(*this, HalfedgeHandle(0), true); }

inline PolyConnectivity::EdgeIter PolyConnectivity::edges_sbegin()
{  return EdgeIter(*this, EdgeHandle(0), true); }

inline PolyConnectivity::ConstEdgeIter PolyConnectivity::edges_sbegin() const
{  return ConstEdgeIter(*this, EdgeHandle(0), true); }

inline PolyConnectivity::FaceIter PolyConnectivity::faces_sbegin()
{  return FaceIter(*this, FaceHandle(0), true); }

inline PolyConnectivity::ConstFaceIter PolyConnectivity::faces_sbegin() const
{  return ConstFaceIter(*this, FaceHandle(0), true); }

inline PolyConnectivity::VertexVertexIter PolyConnectivity::vv_iter(ArrayKernel::VertexHandle _vh)
{  return VertexVertexIter(*this, _vh); }

inline PolyConnectivity::VertexVertexCWIter PolyConnectivity::vv_cwiter(ArrayKernel::VertexHandle _vh)
{  return VertexVertexCWIter(*this, _vh); }

inline PolyConnectivity::VertexVertexCCWIter PolyConnectivity::vv_ccwiter(ArrayKernel::VertexHandle _vh)
{  return VertexVertexCCWIter(*this, _vh); }

inline PolyConnectivity::VertexIHalfedgeIter PolyConnectivity::vih_iter(ArrayKernel::VertexHandle _vh)
{  return VertexIHalfedgeIter(*this, _vh); }

inline PolyConnectivity::VertexIHalfedgeCWIter PolyConnectivity::vih_cwiter(ArrayKernel::VertexHandle _vh)
{  return VertexIHalfedgeCWIter(*this, _vh); }

inline PolyConnectivity::VertexIHalfedgeCCWIter PolyConnectivity::vih_ccwiter(ArrayKernel::VertexHandle _vh)
{  return VertexIHalfedgeCCWIter(*this, _vh); }

inline PolyConnectivity::VertexOHalfedgeIter PolyConnectivity::voh_iter(ArrayKernel::VertexHandle _vh)
{  return VertexOHalfedgeIter(*this, _vh); }

inline PolyConnectivity::VertexOHalfedgeCWIter PolyConnectivity::voh_cwiter(ArrayKernel::VertexHandle _vh)
{  return VertexOHalfedgeCWIter(*this, _vh); }

inline PolyConnectivity::VertexOHalfedgeCCWIter PolyConnectivity::voh_ccwiter(ArrayKernel::VertexHandle _vh)
{  return VertexOHalfedgeCCWIter(*this, _vh); }

inline PolyConnectivity::VertexEdgeIter PolyConnectivity::ve_iter(ArrayKernel::VertexHandle _vh)
{  return VertexEdgeIter(*this, _vh); }

inline PolyConnectivity::VertexEdgeCWIter PolyConnectivity::ve_cwiter(ArrayKernel::VertexHandle _vh)
{  return VertexEdgeCWIter(*this, _vh); }

inline PolyConnectivity::VertexEdgeCCWIter PolyConnectivity::ve_ccwiter(ArrayKernel::VertexHandle _vh)
{  return VertexEdgeCCWIter(*this, _vh); }

inline PolyConnectivity::VertexFaceIter PolyConnectivity::vf_iter(ArrayKernel::VertexHandle _vh)
{  return VertexFaceIter(*this, _vh); }

inline PolyConnectivity::VertexFaceCWIter PolyConnectivity::vf_cwiter(ArrayKernel::VertexHandle _vh)
{  return VertexFaceCWIter(*this, _vh); }

inline PolyConnectivity::VertexFaceCCWIter PolyConnectivity::vf_ccwiter(ArrayKernel::VertexHandle _vh)
{  return VertexFaceCCWIter(*this, _vh); }

inline PolyConnectivity::ConstVertexVertexIter PolyConnectivity::cvv_iter(ArrayKernel::VertexHandle _vh) const
{ return ConstVertexVertexIter(*this, _vh); }

inline PolyConnectivity::ConstVertexVertexCWIter PolyConnectivity::cvv_cwiter(ArrayKernel::VertexHandle _vh) const
{ return ConstVertexVertexCWIter(*this, _vh); }

inline PolyConnectivity::ConstVertexVertexCCWIter PolyConnectivity::cvv_ccwiter(ArrayKernel::VertexHandle _vh) const
{ return ConstVertexVertexCCWIter(*this, _vh); }

inline PolyConnectivity::ConstVertexIHalfedgeIter PolyConnectivity::cvih_iter(ArrayKernel::VertexHandle _vh) const
{ return ConstVertexIHalfedgeIter(*this, _vh); }

inline PolyConnectivity::ConstVertexIHalfedgeCWIter PolyConnectivity::cvih_cwiter(ArrayKernel::VertexHandle _vh) const
{ return ConstVertexIHalfedgeCWIter(*this, _vh); }

inline PolyConnectivity::ConstVertexIHalfedgeCCWIter PolyConnectivity::cvih_ccwiter(ArrayKernel::VertexHandle _vh) const
{ return ConstVertexIHalfedgeCCWIter(*this, _vh); }

inline PolyConnectivity::ConstVertexOHalfedgeIter PolyConnectivity::cvoh_iter(ArrayKernel::VertexHandle _vh) const
{ return ConstVertexOHalfedgeIter(*this, _vh); }

inline PolyConnectivity::ConstVertexOHalfedgeCWIter PolyConnectivity::cvoh_cwiter(ArrayKernel::VertexHandle _vh) const
{ return ConstVertexOHalfedgeCWIter(*this, _vh); }

inline PolyConnectivity::ConstVertexOHalfedgeCCWIter PolyConnectivity::cvoh_ccwiter(ArrayKernel::VertexHandle _vh) const
{ return ConstVertexOHalfedgeCCWIter(*this, _vh); }

inline PolyConnectivity::ConstVertexEdgeIter PolyConnectivity::cve_iter(ArrayKernel::VertexHandle _vh) const
{ return ConstVertexEdgeIter(*this, _vh); }

inline PolyConnectivity::ConstVertexEdgeCWIter PolyConnectivity::cve_cwiter(ArrayKernel::VertexHandle _vh) const
{ return ConstVertexEdgeCWIter(*this, _vh); }

inline PolyConnectivity::ConstVertexEdgeCCWIter PolyConnectivity::cve_ccwiter(ArrayKernel::VertexHandle _vh) const
{ return ConstVertexEdgeCCWIter(*this, _vh); }

inline PolyConnectivity::ConstVertexFaceIter PolyConnectivity::cvf_iter(ArrayKernel::VertexHandle _vh) const
{ return ConstVertexFaceIter(*this, _vh); }

inline PolyConnectivity::ConstVertexFaceCWIter PolyConnectivity::cvf_cwiter(ArrayKernel::VertexHandle _vh) const
{ return ConstVertexFaceCWIter(*this, _vh); }

inline PolyConnectivity::ConstVertexFaceCCWIter PolyConnectivity::cvf_ccwiter(ArrayKernel::VertexHandle _vh) const
{ return ConstVertexFaceCCWIter(*this, _vh); }

inline PolyConnectivity::FaceVertexIter PolyConnectivity::fv_iter(ArrayKernel::FaceHandle _fh)
{ return FaceVertexIter(*this, _fh); }

inline PolyConnectivity::FaceVertexCWIter PolyConnectivity::fv_cwiter(ArrayKernel::FaceHandle _fh)
{ return FaceVertexCWIter(*this, _fh); }

inline PolyConnectivity::FaceVertexCCWIter PolyConnectivity::fv_ccwiter(ArrayKernel::FaceHandle _fh)
{ return FaceVertexCCWIter(*this, _fh); }

inline PolyConnectivity::FaceHalfedgeIter PolyConnectivity::fh_iter(ArrayKernel::FaceHandle _fh)
{ return FaceHalfedgeIter(*this, _fh); }

inline PolyConnectivity::FaceHalfedgeCWIter PolyConnectivity::fh_cwiter(ArrayKernel::FaceHandle _fh)
{ return FaceHalfedgeCWIter(*this, _fh); }

inline PolyConnectivity::FaceHalfedgeCCWIter PolyConnectivity::fh_ccwiter(ArrayKernel::FaceHandle _fh)
{ return FaceHalfedgeCCWIter(*this, _fh); }

inline PolyConnectivity::FaceEdgeIter PolyConnectivity::fe_iter(ArrayKernel::FaceHandle _fh)
{ return FaceEdgeIter(*this, _fh); }

inline PolyConnectivity::FaceEdgeCWIter PolyConnectivity::fe_cwiter(ArrayKernel::FaceHandle _fh)
{ return FaceEdgeCWIter(*this, _fh); }

inline PolyConnectivity::FaceEdgeCCWIter PolyConnectivity::fe_ccwiter(ArrayKernel::FaceHandle _fh)
{ return FaceEdgeCCWIter(*this, _fh); }

inline PolyConnectivity::FaceFaceIter PolyConnectivity::ff_iter(ArrayKernel::FaceHandle _fh)
{ return FaceFaceIter(*this, _fh); }

inline PolyConnectivity::FaceFaceCWIter PolyConnectivity::ff_cwiter(ArrayKernel::FaceHandle _fh)
{ return FaceFaceCWIter(*this, _fh); }

inline PolyConnectivity::FaceFaceCCWIter PolyConnectivity::ff_ccwiter(ArrayKernel::FaceHandle _fh)
{ return FaceFaceCCWIter(*this, _fh); }

inline PolyConnectivity::ConstFaceVertexIter PolyConnectivity::cfv_iter(ArrayKernel::FaceHandle _fh) const
{ return ConstFaceVertexIter(*this, _fh); }

inline PolyConnectivity::ConstFaceVertexCWIter PolyConnectivity::cfv_cwiter(ArrayKernel::FaceHandle _fh) const
{ return ConstFaceVertexCWIter(*this, _fh); }

inline PolyConnectivity::ConstFaceVertexCCWIter PolyConnectivity::cfv_ccwiter(ArrayKernel::FaceHandle _fh) const
{ return ConstFaceVertexCCWIter(*this, _fh); }

inline PolyConnectivity::ConstFaceHalfedgeIter PolyConnectivity::cfh_iter(ArrayKernel::FaceHandle _fh) const
{ return ConstFaceHalfedgeIter(*this, _fh); }

inline PolyConnectivity::ConstFaceHalfedgeCWIter PolyConnectivity::cfh_cwiter(ArrayKernel::FaceHandle _fh) const
{ return ConstFaceHalfedgeCWIter(*this, _fh); }

inline PolyConnectivity::ConstFaceHalfedgeCCWIter PolyConnectivity::cfh_ccwiter(ArrayKernel::FaceHandle _fh) const
{ return ConstFaceHalfedgeCCWIter(*this, _fh); }

inline PolyConnectivity::ConstFaceEdgeIter PolyConnectivity::cfe_iter(ArrayKernel::FaceHandle _fh) const
{ return ConstFaceEdgeIter(*this, _fh); }

inline PolyConnectivity::ConstFaceEdgeCWIter PolyConnectivity::cfe_cwiter(ArrayKernel::FaceHandle _fh) const
{ return ConstFaceEdgeCWIter(*this, _fh); }

inline PolyConnectivity::ConstFaceEdgeCCWIter PolyConnectivity::cfe_ccwiter(ArrayKernel::FaceHandle _fh) const
{ return ConstFaceEdgeCCWIter(*this, _fh); }

inline PolyConnectivity::ConstFaceFaceIter PolyConnectivity::cff_iter(ArrayKernel::FaceHandle _fh) const
{ return ConstFaceFaceIter(*this, _fh); }

inline PolyConnectivity::ConstFaceFaceCWIter PolyConnectivity::cff_cwiter(ArrayKernel::FaceHandle _fh) const
{ return ConstFaceFaceCWIter(*this, _fh); }

inline PolyConnectivity::ConstFaceFaceCCWIter PolyConnectivity::cff_ccwiter(ArrayKernel::FaceHandle _fh) const
{ return ConstFaceFaceCCWIter(*this, _fh); }


inline PolyConnectivity::VertexVertexIter PolyConnectivity::vv_begin(VertexHandle _vh)
{ return VertexVertexIter(*this, _vh); }

inline PolyConnectivity::VertexVertexCWIter PolyConnectivity::vv_cwbegin(VertexHandle _vh)
{ return VertexVertexCWIter(*this, _vh); }

inline PolyConnectivity::VertexVertexCCWIter PolyConnectivity::vv_ccwbegin(VertexHandle _vh)
{ return VertexVertexCCWIter(*this, _vh); }

inline PolyConnectivity::VertexIHalfedgeIter PolyConnectivity::vih_begin(VertexHandle _vh)
{ return VertexIHalfedgeIter(*this, _vh); }

inline PolyConnectivity::VertexIHalfedgeCWIter PolyConnectivity::vih_cwbegin(VertexHandle _vh)
{ return VertexIHalfedgeCWIter(*this, _vh); }

inline PolyConnectivity::VertexIHalfedgeCCWIter PolyConnectivity::vih_ccwbegin(VertexHandle _vh)
{ return VertexIHalfedgeCCWIter(*this, _vh); }

inline PolyConnectivity::VertexOHalfedgeIter PolyConnectivity::voh_begin(VertexHandle _vh)
{ return VertexOHalfedgeIter(*this, _vh); }

inline PolyConnectivity::VertexOHalfedgeCWIter PolyConnectivity::voh_cwbegin(VertexHandle _vh)
{ return VertexOHalfedgeCWIter(*this, _vh); }

inline PolyConnectivity::VertexOHalfedgeCCWIter PolyConnectivity::voh_ccwbegin(VertexHandle _vh)
{ return VertexOHalfedgeCCWIter(*this, _vh); }

inline PolyConnectivity::VertexEdgeIter PolyConnectivity::ve_begin(VertexHandle _vh)
{ return VertexEdgeIter(*this, _vh); }

inline PolyConnectivity::VertexEdgeCWIter PolyConnectivity::ve_cwbegin(VertexHandle _vh)
{ return VertexEdgeCWIter(*this, _vh); }

inline PolyConnectivity::VertexEdgeCCWIter PolyConnectivity::ve_ccwbegin(VertexHandle _vh)
{ return VertexEdgeCCWIter(*this, _vh); }

inline PolyConnectivity::VertexFaceIter PolyConnectivity::vf_begin(VertexHandle _vh)
{ return VertexFaceIter(*this, _vh); }

inline PolyConnectivity::VertexFaceCWIter PolyConnectivity::vf_cwbegin(VertexHandle _vh)
{ return VertexFaceCWIter(*this, _vh); }

inline PolyConnectivity::VertexFaceCCWIter PolyConnectivity::vf_ccwbegin(VertexHandle _vh)
{ return VertexFaceCCWIter(*this, _vh); }


inline PolyConnectivity::ConstVertexVertexIter PolyConnectivity::cvv_begin(VertexHandle _vh) const
{ return ConstVertexVertexIter(*this, _vh); }

inline PolyConnectivity::ConstVertexVertexCWIter PolyConnectivity::cvv_cwbegin(VertexHandle _vh) const
{ return ConstVertexVertexCWIter(*this, _vh); }

inline PolyConnectivity::ConstVertexVertexCCWIter PolyConnectivity::cvv_ccwbegin(VertexHandle _vh) const
{ return ConstVertexVertexCCWIter(*this, _vh); }

inline PolyConnectivity::ConstVertexIHalfedgeIter PolyConnectivity::cvih_begin(VertexHandle _vh) const
{ return ConstVertexIHalfedgeIter(*this, _vh); }

inline PolyConnectivity::ConstVertexIHalfedgeCWIter PolyConnectivity::cvih_cwbegin(VertexHandle _vh) const
{ return ConstVertexIHalfedgeCWIter(*this, _vh); }

inline PolyConnectivity::ConstVertexIHalfedgeCCWIter PolyConnectivity::cvih_ccwbegin(VertexHandle _vh) const
{ return ConstVertexIHalfedgeCCWIter(*this, _vh); }

inline PolyConnectivity::ConstVertexOHalfedgeIter PolyConnectivity::cvoh_begin(VertexHandle _vh) const
{ return ConstVertexOHalfedgeIter(*this, _vh); }

inline PolyConnectivity::ConstVertexOHalfedgeCWIter PolyConnectivity::cvoh_cwbegin(VertexHandle _vh) const
{ return ConstVertexOHalfedgeCWIter(*this, _vh); }

inline PolyConnectivity::ConstVertexOHalfedgeCCWIter PolyConnectivity::cvoh_ccwbegin(VertexHandle _vh) const
{ return ConstVertexOHalfedgeCCWIter(*this, _vh); }

inline PolyConnectivity::ConstVertexEdgeIter PolyConnectivity::cve_begin(VertexHandle _vh) const
{ return ConstVertexEdgeIter(*this, _vh); }

inline PolyConnectivity::ConstVertexEdgeCWIter PolyConnectivity::cve_cwbegin(VertexHandle _vh) const
{ return ConstVertexEdgeCWIter(*this, _vh); }

inline PolyConnectivity::ConstVertexEdgeCCWIter PolyConnectivity::cve_ccwbegin(VertexHandle _vh) const
{ return ConstVertexEdgeCCWIter(*this, _vh); }

inline PolyConnectivity::ConstVertexFaceIter PolyConnectivity::cvf_begin(VertexHandle _vh) const
{ return ConstVertexFaceIter(*this, _vh); }

inline PolyConnectivity::ConstVertexFaceCWIter PolyConnectivity::cvf_cwbegin(VertexHandle _vh) const
{ return ConstVertexFaceCWIter(*this, _vh); }

inline PolyConnectivity::ConstVertexFaceCCWIter PolyConnectivity::cvf_ccwbegin(VertexHandle _vh) const
{ return ConstVertexFaceCCWIter(*this, _vh); }


inline PolyConnectivity::FaceVertexIter PolyConnectivity::fv_begin(FaceHandle _fh)
{ return FaceVertexIter(*this, _fh); }

inline PolyConnectivity::FaceVertexCWIter PolyConnectivity::fv_cwbegin(FaceHandle _fh)
{ return FaceVertexCWIter(*this, _fh); }

inline PolyConnectivity::FaceVertexCCWIter PolyConnectivity::fv_ccwbegin(FaceHandle _fh)
{ return FaceVertexCCWIter(*this, _fh); }

inline PolyConnectivity::FaceHalfedgeIter PolyConnectivity::fh_begin(FaceHandle _fh)
{ return FaceHalfedgeIter(*this, _fh); }

inline PolyConnectivity::FaceHalfedgeCWIter PolyConnectivity::fh_cwbegin(FaceHandle _fh)
{ return FaceHalfedgeCWIter(*this, _fh); }

inline PolyConnectivity::FaceHalfedgeCCWIter PolyConnectivity::fh_ccwbegin(FaceHandle _fh)
{ return FaceHalfedgeCCWIter(*this, _fh); }

inline PolyConnectivity::FaceEdgeIter PolyConnectivity::fe_begin(FaceHandle _fh)
{ return FaceEdgeIter(*this, _fh); }

inline PolyConnectivity::FaceEdgeCWIter PolyConnectivity::fe_cwbegin(FaceHandle _fh)
{ return FaceEdgeCWIter(*this, _fh); }

inline PolyConnectivity::FaceEdgeCCWIter PolyConnectivity::fe_ccwbegin(FaceHandle _fh)
{ return FaceEdgeCCWIter(*this, _fh); }

inline PolyConnectivity::FaceFaceIter PolyConnectivity::ff_begin(FaceHandle _fh)
{ return FaceFaceIter(*this, _fh); }

inline PolyConnectivity::FaceFaceCWIter PolyConnectivity::ff_cwbegin(FaceHandle _fh)
{ return FaceFaceCWIter(*this, _fh); }

inline PolyConnectivity::FaceFaceCCWIter PolyConnectivity::ff_ccwbegin(FaceHandle _fh)
{ return FaceFaceCCWIter(*this, _fh); }

inline PolyConnectivity::HalfedgeLoopIter PolyConnectivity::hl_begin(HalfedgeHandle _heh)
{ return HalfedgeLoopIter(*this, _heh); }

inline PolyConnectivity::HalfedgeLoopCWIter PolyConnectivity::hl_cwbegin(HalfedgeHandle _heh)
{ return HalfedgeLoopCWIter(*this, _heh); }

inline PolyConnectivity::HalfedgeLoopCCWIter PolyConnectivity::hl_ccwbegin(HalfedgeHandle _heh)
{ return HalfedgeLoopCCWIter(*this, _heh); }


inline PolyConnectivity::ConstFaceVertexIter PolyConnectivity::cfv_begin(FaceHandle _fh) const
{ return ConstFaceVertexIter(*this, _fh); }

inline PolyConnectivity::ConstFaceVertexCWIter PolyConnectivity::cfv_cwbegin(FaceHandle _fh) const
{ return ConstFaceVertexCWIter(*this, _fh); }

inline PolyConnectivity::ConstFaceVertexCCWIter PolyConnectivity::cfv_ccwbegin(FaceHandle _fh) const
{ return ConstFaceVertexCCWIter(*this, _fh); }

inline PolyConnectivity::ConstFaceHalfedgeIter PolyConnectivity::cfh_begin(FaceHandle _fh) const
{ return ConstFaceHalfedgeIter(*this, _fh); }

inline PolyConnectivity::ConstFaceHalfedgeCWIter PolyConnectivity::cfh_cwbegin(FaceHandle _fh) const
{ return ConstFaceHalfedgeCWIter(*this, _fh); }

inline PolyConnectivity::ConstFaceHalfedgeCCWIter PolyConnectivity::cfh_ccwbegin(FaceHandle _fh) const
{ return ConstFaceHalfedgeCCWIter(*this, _fh); }

inline PolyConnectivity::ConstFaceEdgeIter PolyConnectivity::cfe_begin(FaceHandle _fh) const
{ return ConstFaceEdgeIter(*this, _fh); }

inline PolyConnectivity::ConstFaceEdgeCWIter PolyConnectivity::cfe_cwbegin(FaceHandle _fh) const
{ return ConstFaceEdgeCWIter(*this, _fh); }

inline PolyConnectivity::ConstFaceEdgeCCWIter PolyConnectivity::cfe_ccwbegin(FaceHandle _fh) const
{ return ConstFaceEdgeCCWIter(*this, _fh); }

inline PolyConnectivity::ConstFaceFaceIter PolyConnectivity::cff_begin(FaceHandle _fh) const
{ return ConstFaceFaceIter(*this, _fh); }

inline PolyConnectivity::ConstFaceFaceCWIter PolyConnectivity::cff_cwbegin(FaceHandle _fh) const
{ return ConstFaceFaceCWIter(*this, _fh); }

inline PolyConnectivity::ConstFaceFaceCCWIter PolyConnectivity::cff_ccwbegin(FaceHandle _fh) const
{ return ConstFaceFaceCCWIter(*this, _fh); }

inline PolyConnectivity::ConstHalfedgeLoopIter PolyConnectivity::chl_begin(HalfedgeHandle _heh) const
{ return ConstHalfedgeLoopIter(*this, _heh); }

inline PolyConnectivity::ConstHalfedgeLoopCWIter PolyConnectivity::chl_cwbegin(HalfedgeHandle _heh) const
{ return ConstHalfedgeLoopCWIter(*this, _heh); }

inline PolyConnectivity::ConstHalfedgeLoopCCWIter PolyConnectivity::chl_ccwbegin(HalfedgeHandle _heh) const
{ return ConstHalfedgeLoopCCWIter(*this, _heh); }

// 'end' circulators

inline PolyConnectivity::VertexVertexIter PolyConnectivity::vv_end(VertexHandle _vh)
{ return VertexVertexIter(*this, _vh, true); }

inline PolyConnectivity::VertexVertexCWIter PolyConnectivity::vv_cwend(VertexHandle _vh)
{ return VertexVertexCWIter(*this, _vh, true); }

inline PolyConnectivity::VertexVertexCCWIter PolyConnectivity::vv_ccwend(VertexHandle _vh)
{ return VertexVertexCCWIter(*this, _vh, true); }

inline PolyConnectivity::VertexIHalfedgeIter PolyConnectivity::vih_end(VertexHandle _vh)
{ return VertexIHalfedgeIter(*this, _vh, true); }

inline PolyConnectivity::VertexIHalfedgeCWIter PolyConnectivity::vih_cwend(VertexHandle _vh)
{ return VertexIHalfedgeCWIter(*this, _vh, true); }

inline PolyConnectivity::VertexIHalfedgeCCWIter PolyConnectivity::vih_ccwend(VertexHandle _vh)
{ return VertexIHalfedgeCCWIter(*this, _vh, true); }

inline PolyConnectivity::VertexOHalfedgeIter PolyConnectivity::voh_end(VertexHandle _vh)
{ return VertexOHalfedgeIter(*this, _vh, true); }

inline PolyConnectivity::VertexOHalfedgeCWIter PolyConnectivity::voh_cwend(VertexHandle _vh)
{ return VertexOHalfedgeCWIter(*this, _vh, true); }

inline PolyConnectivity::VertexOHalfedgeCCWIter PolyConnectivity::voh_ccwend(VertexHandle _vh)
{ return VertexOHalfedgeCCWIter(*this, _vh, true); }

inline PolyConnectivity::VertexEdgeIter PolyConnectivity::ve_end(VertexHandle _vh)
{ return VertexEdgeIter(*this, _vh, true); }

inline PolyConnectivity::VertexEdgeCWIter PolyConnectivity::ve_cwend(VertexHandle _vh)
{ return VertexEdgeCWIter(*this, _vh, true); }

inline PolyConnectivity::VertexEdgeCCWIter PolyConnectivity::ve_ccwend(VertexHandle _vh)
{ return VertexEdgeCCWIter(*this, _vh, true); }

inline PolyConnectivity::VertexFaceIter PolyConnectivity::vf_end(VertexHandle _vh)
{ return VertexFaceIter(*this, _vh, true); }

inline PolyConnectivity::VertexFaceCWIter PolyConnectivity::vf_cwend(VertexHandle _vh)
{ return VertexFaceCWIter(*this, _vh, true); }

inline PolyConnectivity::VertexFaceCCWIter PolyConnectivity::vf_ccwend(VertexHandle _vh)
{ return VertexFaceCCWIter(*this, _vh, true); }


inline PolyConnectivity::ConstVertexVertexIter PolyConnectivity::cvv_end(VertexHandle _vh) const
{ return ConstVertexVertexIter(*this, _vh, true); }

inline PolyConnectivity::ConstVertexVertexCWIter PolyConnectivity::cvv_cwend(VertexHandle _vh) const
{ return ConstVertexVertexCWIter(*this, _vh, true); }

inline PolyConnectivity::ConstVertexVertexCCWIter PolyConnectivity::cvv_ccwend(VertexHandle _vh) const
{ return ConstVertexVertexCCWIter(*this, _vh, true); }

inline PolyConnectivity::ConstVertexIHalfedgeIter PolyConnectivity::cvih_end(VertexHandle _vh) const
{ return ConstVertexIHalfedgeIter(*this, _vh, true); }

inline PolyConnectivity::ConstVertexIHalfedgeCWIter PolyConnectivity::cvih_cwend(VertexHandle _vh) const
{ return ConstVertexIHalfedgeCWIter(*this, _vh, true); }

inline PolyConnectivity::ConstVertexIHalfedgeCCWIter PolyConnectivity::cvih_ccwend(VertexHandle _vh) const
{ return ConstVertexIHalfedgeCCWIter(*this, _vh, true); }

inline PolyConnectivity::ConstVertexOHalfedgeIter PolyConnectivity::cvoh_end(VertexHandle _vh) const
{ return ConstVertexOHalfedgeIter(*this, _vh, true); }

inline PolyConnectivity::ConstVertexOHalfedgeCWIter PolyConnectivity::cvoh_cwend(VertexHandle _vh) const
{ return ConstVertexOHalfedgeCWIter(*this, _vh, true); }

inline PolyConnectivity::ConstVertexOHalfedgeCCWIter PolyConnectivity::cvoh_ccwend(VertexHandle _vh) const
{ return ConstVertexOHalfedgeCCWIter(*this, _vh, true); }

inline PolyConnectivity::ConstVertexEdgeIter PolyConnectivity::cve_end(VertexHandle _vh) const
{ return ConstVertexEdgeIter(*this, _vh, true); }

inline PolyConnectivity::ConstVertexEdgeCWIter PolyConnectivity::cve_cwend(VertexHandle _vh) const
{ return ConstVertexEdgeCWIter(*this, _vh, true); }

inline PolyConnectivity::ConstVertexEdgeCCWIter PolyConnectivity::cve_ccwend(VertexHandle _vh) const
{ return ConstVertexEdgeCCWIter(*this, _vh, true); }

inline PolyConnectivity::ConstVertexFaceIter PolyConnectivity::cvf_end(VertexHandle _vh) const
{ return ConstVertexFaceIter(*this, _vh, true); }

inline PolyConnectivity::ConstVertexFaceCWIter PolyConnectivity::cvf_cwend(VertexHandle _vh) const
{ return ConstVertexFaceCWIter(*this, _vh, true); }

inline PolyConnectivity::ConstVertexFaceCCWIter PolyConnectivity::cvf_ccwend(VertexHandle _vh) const
{ return ConstVertexFaceCCWIter(*this, _vh, true); }


inline PolyConnectivity::FaceVertexIter PolyConnectivity::fv_end(FaceHandle _fh)
{ return FaceVertexIter(*this, _fh, true); }

inline PolyConnectivity::FaceVertexCWIter PolyConnectivity::fv_cwend(FaceHandle _fh)
{ return FaceVertexCWIter(*this, _fh, true); }

inline PolyConnectivity::FaceVertexCCWIter PolyConnectivity::fv_ccwend(FaceHandle _fh)
{ return FaceVertexCCWIter(*this, _fh, true); }

inline PolyConnectivity::FaceHalfedgeIter PolyConnectivity::fh_end(FaceHandle _fh)
{ return FaceHalfedgeIter(*this, _fh, true); }

inline PolyConnectivity::FaceHalfedgeCWIter PolyConnectivity::fh_cwend(FaceHandle _fh)
{ return FaceHalfedgeCWIter(*this, _fh, true); }

inline PolyConnectivity::FaceHalfedgeCCWIter PolyConnectivity::fh_ccwend(FaceHandle _fh)
{ return FaceHalfedgeCCWIter(*this, _fh, true); }

inline PolyConnectivity::FaceEdgeIter PolyConnectivity::fe_end(FaceHandle _fh)
{ return FaceEdgeIter(*this, _fh, true); }

inline PolyConnectivity::FaceEdgeCWIter PolyConnectivity::fe_cwend(FaceHandle _fh)
{ return FaceEdgeCWIter(*this, _fh, true); }

inline PolyConnectivity::FaceEdgeCCWIter PolyConnectivity::fe_ccwend(FaceHandle _fh)
{ return FaceEdgeCCWIter(*this, _fh, true); }

inline PolyConnectivity::FaceFaceIter PolyConnectivity::ff_end(FaceHandle _fh)
{ return FaceFaceIter(*this, _fh, true); }

inline PolyConnectivity::FaceFaceCWIter PolyConnectivity::ff_cwend(FaceHandle _fh)
{ return FaceFaceCWIter(*this, _fh, true); }

inline PolyConnectivity::FaceFaceCCWIter PolyConnectivity::ff_ccwend(FaceHandle _fh)
{ return FaceFaceCCWIter(*this, _fh, true); }

inline PolyConnectivity::HalfedgeLoopIter PolyConnectivity::hl_end(HalfedgeHandle _heh)
{ return HalfedgeLoopIter(*this, _heh, true); }

inline PolyConnectivity::HalfedgeLoopCWIter PolyConnectivity::hl_cwend(HalfedgeHandle _heh)
{ return HalfedgeLoopCWIter(*this, _heh, true); }

inline PolyConnectivity::HalfedgeLoopCCWIter PolyConnectivity::hl_ccwend(HalfedgeHandle _heh)
{ return HalfedgeLoopCCWIter(*this, _heh, true); }


inline PolyConnectivity::ConstFaceVertexIter PolyConnectivity::cfv_end(FaceHandle _fh) const
{ return ConstFaceVertexIter(*this, _fh, true); }

inline PolyConnectivity::ConstFaceVertexCWIter PolyConnectivity::cfv_cwend(FaceHandle _fh) const
{ return ConstFaceVertexCWIter(*this, _fh, true); }

inline PolyConnectivity::ConstFaceVertexCCWIter PolyConnectivity::cfv_ccwend(FaceHandle _fh) const
{ return ConstFaceVertexCCWIter(*this, _fh, true); }

inline PolyConnectivity::ConstFaceHalfedgeIter PolyConnectivity::cfh_end(FaceHandle _fh) const
{ return ConstFaceHalfedgeIter(*this, _fh, true); }

inline PolyConnectivity::ConstFaceHalfedgeCWIter PolyConnectivity::cfh_cwend(FaceHandle _fh) const
{ return ConstFaceHalfedgeCWIter(*this, _fh, true); }

inline PolyConnectivity::ConstFaceHalfedgeCCWIter PolyConnectivity::cfh_ccwend(FaceHandle _fh) const
{ return ConstFaceHalfedgeCCWIter(*this, _fh, true); }

inline PolyConnectivity::ConstFaceEdgeIter PolyConnectivity::cfe_end(FaceHandle _fh) const
{ return ConstFaceEdgeIter(*this, _fh, true); }

inline PolyConnectivity::ConstFaceEdgeCWIter PolyConnectivity::cfe_cwend(FaceHandle _fh) const
{ return ConstFaceEdgeCWIter(*this, _fh, true); }

inline PolyConnectivity::ConstFaceEdgeCCWIter PolyConnectivity::cfe_ccwend(FaceHandle _fh) const
{ return ConstFaceEdgeCCWIter(*this, _fh, true); }

inline PolyConnectivity::ConstFaceFaceIter PolyConnectivity::cff_end(FaceHandle _fh) const
{ return ConstFaceFaceIter(*this, _fh, true); }

inline PolyConnectivity::ConstFaceFaceCWIter PolyConnectivity::cff_cwend(FaceHandle _fh) const
{ return ConstFaceFaceCWIter(*this, _fh, true); }

inline PolyConnectivity::ConstFaceFaceCCWIter PolyConnectivity::cff_ccwend(FaceHandle _fh) const
{ return ConstFaceFaceCCWIter(*this, _fh, true); }

inline PolyConnectivity::ConstHalfedgeLoopIter PolyConnectivity::chl_end(HalfedgeHandle _heh) const
{ return ConstHalfedgeLoopIter(*this, _heh, true); }

inline PolyConnectivity::ConstHalfedgeLoopCWIter PolyConnectivity::chl_cwend(HalfedgeHandle _heh) const
{ return ConstHalfedgeLoopCWIter(*this, _heh, true); }

inline PolyConnectivity::ConstHalfedgeLoopCCWIter PolyConnectivity::chl_ccwend(HalfedgeHandle _heh) const
{ return ConstHalfedgeLoopCCWIter(*this, _heh, true); }


}//namespace OpenMesh


#endif//OPENMESH_POLYCONNECTIVITY_HH
