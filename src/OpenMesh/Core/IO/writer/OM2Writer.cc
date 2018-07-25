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

#include <OpenMesh/Core/System/config.h>
// -------------------- STL
#if defined( OM_CC_MIPS )
  #include <time.h>
  #include <string.h>
#else
  #include <ctime>
  #include <cstring>
#endif

#include <fstream>
#include <vector>

// -------------------- OpenMesh
#include <OpenMesh/Core/IO/OMFormat.hh>
#include <OpenMesh/Core/IO/exporter/BaseExporter.hh>
#include <OpenMesh/Core/IO/writer/OM2Writer.hh>

//=== NAMESPACES ==============================================================


namespace OpenMesh {
namespace IO {


//=== INSTANCIATE =============================================================


// register the OMLoader singleton with MeshLoader
_OM2Writer_  __OM2WriterInstance;
_OM2Writer_& OM2Writer() { return __OM2WriterInstance; }


//=== IMPLEMENTATION ==========================================================

_OM2Writer_::
_OM2Writer_()
{
  IOManager().register_module(this);
}


bool
_OM2Writer_::write(const std::string& _filename, BaseExporter& _be,
                   Options _opt, std::streamsize /*_precision*/) const
{
  // check whether exporter can give us an OpenMesh BaseKernel
  if (!_be.kernel()) return false;

  _opt += Options::Binary; // only binary format supported

  std::ofstream ofs(_filename.c_str(), std::ios::binary);

  // check if file is open
  if (!ofs.is_open())
  {
    omerr() << "[OMWriter] : cannot open file " << _filename << std::endl;
    return false;
  }

  // call stream save method
  bool rc = write(ofs, _be, _opt);

  // close filestream
  ofs.close();

  // return success/failure notice
  return rc;
}


//-----------------------------------------------------------------------------

bool
_OM2Writer_::write(std::ostream& _os, BaseExporter& _be, Options _opt, std::streamsize /*_precision*/) const
{
//   std::clog << "[OMWriter]::write( stream )\n";

  // check exporter features
  if ( !check( _be, _opt ) )
  {
    omerr() << "[OMWriter]: exporter does not support wanted feature!\n";
    return false;
  }

  // Maybe an ascii version will be implemented in the future.
  // For now, support only a binary format
  if ( !_opt.check( Options::Binary ) )
    _opt += Options::Binary;

  // Ignore LSB/MSB bit. Always store in LSB (little endian)
  _opt += Options::LSB;
  _opt -= Options::MSB;

  return write_binary(_os, _be, _opt);
}


//-----------------------------------------------------------------------------

bool _OM2Writer_::write_binary(std::ostream& _os, BaseExporter& _be,
                               Options _opt) const
{
    throw std::logic_error("Not implementable right now.");
}

// ----------------------------------------------------------------------------

size_t _OM2Writer_::binary_size(BaseExporter& /* _be */, Options /* _opt */) const
{
  // !!!TODO!!!

  return -1;
}

//=============================================================================
} // namespace IO
} // namespace OpenMesh
//=============================================================================
