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


//== INCLUDES =================================================================


//STL
#include <fstream>
#include <limits>

// OpenMesh
#include <OpenMesh/Core/IO/BinaryHelper.hh>
#include <OpenMesh/Core/IO/writer/OBJWriter.hh>
#include <OpenMesh/Core/IO/IOManager.hh>
#include <OpenMesh/Core/Utils/color_cast.hh>

//=== NAMESPACES ==============================================================


namespace OpenMesh {
namespace IO {


//=== INSTANCIATE =============================================================


// register the OBJLoader singleton with MeshLoader
_OBJWriter_  __OBJWriterinstance;
_OBJWriter_& OBJWriter() { return __OBJWriterinstance; }


//=== IMPLEMENTATION ==========================================================


_OBJWriter_::_OBJWriter_() { IOManager().register_module(this); }


//-----------------------------------------------------------------------------


bool
_OBJWriter_::
write(const std::string& _filename, BaseExporter& _be, Options _opt, std::streamsize _precision) const
{
  std::fstream out(_filename.c_str(), std::ios_base::out );

  if (!out)
  {
    omerr() << "[OBJWriter] : cannot open file "
	  << _filename << std::endl;
    return false;
  }

  out.precision(_precision);

  {
#if defined(WIN32)
    std::string::size_type dot = _filename.find_last_of("\\/");
#else
    std::string::size_type dot = _filename.rfind("/");
#endif

    if (dot == std::string::npos){
      path_ = "./";
      objName_ = _filename;
    }else{
      path_ = _filename.substr(0,dot+1);
      objName_ = _filename.substr(dot+1);
    }

    //remove the file extension
    dot = objName_.find_last_of(".");

    if(dot != std::string::npos)
      objName_ = objName_.substr(0,dot);
  }

  bool result = write(out, _be, _opt, _precision);

  out.close();
  return result;
}

//-----------------------------------------------------------------------------

size_t _OBJWriter_::getCachedMaterial(const Material& _material) const
{
  for (size_t i=0; i < material_.size(); i++)
    if(material_[i] == _material)
      return i;

  //not found add new material
  material_.push_back( _material );
  return material_.size()-1;
}

//-----------------------------------------------------------------------------

_OBJWriter_::Material _OBJWriter_::getMaterialDescription(FaceHandle _f, BaseExporter& _be, Options _opt) const
{
  Material mat;
  mat.hasColor = _opt.face_has_color();
  if (mat.hasColor) {
    //color with alpha
    if ( _opt.color_has_alpha() ){
      mat.color = color_cast<OpenMesh::Vec4f>(_be.colorA(_f));
    }else{
      //and without alpha
      mat.color = color_cast<OpenMesh::Vec4f>(_be.color(_f));
    }
  }

  mat.hasTextureIndex = _opt.face_has_texture_index() && (_be.face_texindex(_f) > 0);
  if (mat.hasTextureIndex) {
    mat.textureIndex = _be.face_texindex(_f);
  }
  return mat;
}

//-----------------------------------------------------------------------------

bool _OBJWriter_::writeMaterial(std::ostream& _out, BaseExporter& _be, Options _opt) const
{
  material_.clear();

  //iterate over faces
  for (size_t i=0, nF=_be.n_faces(); i<nF; ++i)
  {
    Material mat = getMaterialDescription(FaceHandle(int(i)), _be, _opt);
    if (mat.is_valid()) {
      // Create material
      getCachedMaterial(mat);
    }
  }

  //write the materials
  for (size_t i=0; i < material_.size(); i++) {
    const Material& m = material_[i];
    _out << "newmtl " << "mat" << i << '\n';
    if (m.hasColor) {
      _out << "Ka 0.5000 0.5000 0.5000" << '\n';
      _out << "Kd " << m.color[0] << ' ' << m.color[1] << ' ' << m.color[2] << '\n';
       if (m.color[3] < 1.0)
         _out << "Tr " << m.color[3] << '\n';
      _out << "illum 1" << '\n';
    }
    if (m.hasTextureIndex) {
       _out << "map_Kd "<<_be.texture_name(m.textureIndex)<< std::endl;
    }
    _out  << std::endl;
  }

  return true;
}

//-----------------------------------------------------------------------------


bool
_OBJWriter_::
write(std::ostream& _out, BaseExporter& _be, Options _opt, std::streamsize _precision) const
{
  bool useMatrial = false;

  omlog() << "[OBJWriter] : write file\n";

  _out.precision(_precision);

  // check exporter features
  if (!check( _be, _opt))
     return false;

  // No binary mode for OBJ
  if ( _opt.check(Options::Binary) ) {
    omout() << "[OBJWriter] : Warning, Binary mode requested for OBJ Writer (No support for Binary mode), falling back to standard." <<  std::endl;
  }

  // check for unsupported writer features
  if (_opt.check(Options::FaceNormal) ) {
    omerr() << "[OBJWriter] : FaceNormal not supported by OBJ Writer" <<  std::endl;
    return false;
  }

  // check for unsupported writer features
  if (_opt.check(Options::VertexColor) ) {
    omerr() << "[OBJWriter] : VertexColor not supported by OBJ Writer" <<  std::endl;
    return false;
  }

  //create material file if needed
  if ( _opt.check(Options::FaceColor) || _opt.check(Options::FaceTextureIndex) ){

    std::string matFile = path_ + objName_ + ".mat";

    std::fstream matStream(matFile.c_str(), std::ios_base::out );

    if (!matStream)
    {
      omerr() << "[OBJWriter] : cannot write material file " << matFile << std::endl;

    }else{
      useMatrial = writeMaterial(matStream, _be, _opt);

      matStream.close();
    }
  }

  // header
  _out << "# " << _be.n_vertices() << " vertices, ";
  _out << _be.n_faces() << " faces" << '\n';

  // material file
  if (useMatrial)
    _out << "mtllib " << objName_ << ".mat" << '\n';

  std::map<Vec2f,int> texMap;
  //collect Texturevertices from halfedges
  if(_opt.check(Options::HalfedgeTexCoord))
  {
    std::vector<Vec2f> texCoords;
    //add all texCoords to map
    unsigned int num = _be.get_halfedge_texcoords(texCoords);
    for(unsigned int i = 0; i < num ; ++i)
    {
      texMap[texCoords[i]] = i;
    }
  }

  //collect Texture coordinates from vertices
  if(_opt.check(Options::VertexTexCoord))
  {
    for (size_t i=0, nV=_be.n_vertices(); i<nV; ++i)
    {
      VertexHandle vh(static_cast<int>(i));
      Vec2f t = _be.texcoord(vh);
      texMap[t] = static_cast<int>(i);
    }
  }

  std::vector<VertexHandle> vhandles;

  std::map<Vec3f,int> normalMap;
  //collect halfedge normals
  if(_opt.check(Options::HalfedgeNormal))
  {
    std::vector<Vec3f> normals;
    for (std::size_t f = 0; f < _be.n_faces(); ++f)
    {
      FaceHandle fh(f);
      _be.get_vhandles(fh, vhandles);
      for (std::size_t j=0; j< vhandles.size(); ++j)
      {
        const OpenMesh::Vec3f& n = _be.normal(_be.getHeh(fh, vhandles[j]));
        normalMap[n] = -1;
      }
    }
  }
  //collect vertex normals
  else if(_opt.check(Options::VertexNormal))
  {
    for (size_t i=0, nV = _be.n_vertices(); i<nV; ++i)
    {
      VertexHandle vh(static_cast<int>(i));
      Vec3f n = _be.normal(vh);
      normalMap[n] = static_cast<int>(i);
    }
  }


  // assign each texcoord in the map its id
  // and write the vt entries
  if(_opt.check(Options::VertexTexCoord) || _opt.check(Options::HalfedgeTexCoord))
  {
    int texCount = 0;
    for(std::map<Vec2f,int>::iterator it = texMap.begin(); it != texMap.end() ; ++it)
    {
      _out << "vt " << it->first[0] << " " << it->first[1] << '\n';
      it->second = ++texCount;
    }
  }

  // assign each normal in the map its id
  // and write the vn entries
  if(_opt.check(Options::VertexNormal) || _opt.check(Options::HalfedgeNormal))
  {
    int normalId = 0;
    for(std::map<Vec3f,int>::iterator it = normalMap.begin(); it != normalMap.end() ; ++it)
    {
      _out << "vn " << it->first[0] << " " << it->first[1] << " " << it->first[2] << '\n';
      it->second = ++normalId;
    }
  }


  // vertex data (point, normals, texcoords)
  for (std::size_t i=0, nV=_be.n_vertices(); i<nV; ++i)
  {
    VertexHandle vh(static_cast<int>(i));
    Vec3f v  = _be.point(vh);
    _out << "v " << v[0] <<" "<< v[1] <<" "<< v[2] << '\n';
  }

  size_t lastMaterialIndex = std::numeric_limits<std::size_t>::max();

  // we do not want to write seperators if we only write vertex indices
  bool onlyVertices =    !_opt.check(Options::VertexTexCoord)
                      && !(_opt.check(Options::VertexNormal) || _opt.check(Options::HalfedgeNormal))
                      && !_opt.check(Options::HalfedgeTexCoord);


  // faces (indices starting at 1 not 0)
  for (std::size_t i=0, nF=_be.n_faces(); i<nF; ++i)
  {
    if (useMatrial) {
      size_t materialIndex = std::numeric_limits<std::size_t>::max();

      Material material = getMaterialDescription(FaceHandle(int(i)), _be, _opt);
      if (material.is_valid()) {
        materialIndex = getCachedMaterial(material);
        // if we are ina a new material block, specify in the file which material to use
        if(lastMaterialIndex != materialIndex) {
          _out << "usemtl mat" << materialIndex << '\n';
          lastMaterialIndex = materialIndex;
        }
      }
    }

    _out << "f";

    _be.get_vhandles(FaceHandle(int(i)), vhandles);

    for (std::size_t j=0; j< vhandles.size(); ++j)
    {

      // Write vertex index
      unsigned int idx = vhandles[j].idx() + 1;
      _out << " " << idx;

      if (!onlyVertices) {
        // write separator
        _out << "/" ;

        //write texCoords index from halfedge
        if(_opt.check(Options::HalfedgeTexCoord) )
        {
          bool supplyCoord = (!_opt.face_has_texture_index()) || (_be.face_texindex(FaceHandle(int(i))) > 0);
          if (supplyCoord)
          {
            _out << texMap[_be.texcoord(_be.getHeh(FaceHandle(int(i)),vhandles[j]))];
          }
        }

        else
        {
          // write vertex texture coordinate index
          if (_opt.check(Options::VertexTexCoord))
            _out  << texMap[_be.texcoord(vhandles[j])];
        }

        if (_opt.check(Options::HalfedgeNormal))
        {
            // write separator
            _out << "/" ;
        	_out << normalMap[_be.normal(_be.getHeh(FaceHandle(int(i)), vhandles[j]))];
        }
        // write vertex normal index
        else if ( _opt.check(Options::VertexNormal) ) {
          // write separator
          _out << "/" ;
          _out << normalMap[_be.normal(vhandles[j])];
        }
      }
    }

    _out << '\n';
  }

  material_.clear();

  return true;
}


//=============================================================================
} // namespace IO
} // namespace OpenMesh
//=============================================================================
