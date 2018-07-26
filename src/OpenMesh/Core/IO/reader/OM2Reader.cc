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

// STL
#include <fstream>
#include <istream>
#include <vector>

// OpenMesh
#include <OpenMesh/Core/IO/OM2Format.hh>
#include <OpenMesh/Core/IO/OMFormat.hh>
#include <OpenMesh/Core/IO/reader/OMReader.hh>
#include <OpenMesh/Core/IO/reader/OM2Reader.hh>
#include <OpenMesh/Core/System/config.h>
#include <OpenMesh/Core/System/omstream.hh>
#include <OpenMesh/Core/Utils/Endian.hh>

//=== NAMESPACES ==============================================================

namespace OpenMesh {
namespace IO {

//=== INSTANCIATE =============================================================

// register the OM2Reader singleton with MeshReader
_OM2Reader_ __OM2ReaderInstance;
_OM2Reader_ &OM2Reader() { return __OM2ReaderInstance; }

//=== IMPLEMENTATION ==========================================================

_OM2Reader_::_OM2Reader_() { IOManager().register_module(this); }

//-----------------------------------------------------------------------------

bool _OM2Reader_::read(const std::string &_filename, BaseImporter &_bi,
                       Options &_opt) {
    // check whether importer can give us an OpenMesh BaseKernel
    if (!_bi.kernel()) return false;

    _opt += Options::Binary; // only binary format supported!

    // Open file
    std::ifstream ifs(_filename.c_str(), std::ios::binary);

    /* Clear formatting flag skipws (Skip whitespaces). If set, operator>> will
     * skip bytes set to whitespace chars (e.g. 0x20 bytes) in
     * Property<bool>::restore.
     */
    ifs.unsetf(std::ios::skipws);

    if (!ifs.is_open() || !ifs.good()) {
        omerr() << "[OM2Reader] : cannot not open file " << _filename
                << std::endl;
        return false;
    }

    // Pass stream to read method, remember result
    bool result = read(ifs, _bi, _opt);

    // close input stream
    ifs.close();

    return result;
}

//-----------------------------------------------------------------------------

bool _OM2Reader_::read(std::istream &_is, BaseImporter &_bi, Options &_opt) {
    // check whether importer can give us an OpenMesh BaseKernel
    if (!_bi.kernel()) return false;

    _opt += Options::Binary; // only binary format supported!

    if (!_is.good()) {
        omerr() << "[OMReader] : cannot read from stream " << std::endl;
        return false;
    }

    // Pass stream to read method, remember result
    bool result = read_binary(_is, _bi, _opt);

    if (result) _opt += Options::Binary;

    return result;
}

//-----------------------------------------------------------------------------

bool _OM2Reader_::read_ascii(std::istream & /* _is */, BaseImporter & /* _bi */,
                             Options & /* _opt */) const {
    // not supported yet!
    return false;
}

//-----------------------------------------------------------------------------

bool _OM2Reader_::read_binary(std::istream &_is, BaseImporter &_bi,
                              Options &_opt) const {
    using Opts = OpenMesh::IO::Options;

    OM2Format::TypeInfo::ExtendedOptions eopts = OM2Format::TypeInfo::readPreamble(_is, *_bi.kernel());

    if (eopts.has_halfedge_texcoords1D()) _bi.request_halfedge_texcoords1D();
    if (eopts.has_halfedge_texcoords2D()) _bi.request_halfedge_texcoords2D();
    if (eopts.has_halfedge_texcoords3D()) _bi.request_halfedge_texcoords3D();

    if (eopts.has_vertex_texcoords1D()) _bi.request_vertex_texcoords1D();
    if (eopts.has_vertex_texcoords2D()) _bi.request_vertex_texcoords2D();
    if (eopts.has_vertex_texcoords3D()) _bi.request_vertex_texcoords3D();

    _opt = eopts.om();

    if (_opt.vertex_has_normal()   ) _bi.request_vertex_normals();
    if (_opt.vertex_has_color()    ) _bi.request_vertex_colors();
    if (_opt.edge_has_color()      ) _bi.request_edge_colors();
    if (_opt.face_has_normal()     ) _bi.request_face_normals();
    if (_opt.face_has_color()      ) _bi.request_face_colors();

    return __OMReaderInstance.read(_is, _bi, _opt);
}

//-----------------------------------------------------------------------------

bool _OM2Reader_::can_u_read(const std::string &_filename) const {
    // !!! Assuming BaseReader::can_u_parse( std::string& )
    // does not call BaseReader::read_magic()!!!
    if (this->BaseReader::can_u_read(_filename)) {
        std::ifstream ifile(_filename.c_str());
        if (ifile && can_u_read(ifile)) return true;
    }
    return false;
}

//-----------------------------------------------------------------------------

bool _OM2Reader_::can_u_read(std::istream &_is) const {
    char tip[4];
    _is.read(tip, 4);

    for (char c : tip) _is.putback(c);

    return OM2Format::TypeInfo::checkMagic(tip);
}

//-----------------------------------------------------------------------------

//=============================================================================
} // namespace IO
} // namespace OpenMesh
//=============================================================================
