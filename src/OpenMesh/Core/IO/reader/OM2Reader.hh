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
//  Implements a reader module for OM2 (aka PROM) files
//
//=============================================================================


#ifndef __OM2READER_HH__
#define __OM2READER_HH__


//=== INCLUDES ================================================================

// OpenMesh
#include <OpenMesh/Core/System/config.h>
#include <OpenMesh/Core/Utils/SingletonT.hh>
#include <OpenMesh/Core/IO/OMFormat.hh>
#include <OpenMesh/Core/IO/IOManager.hh>
#include <OpenMesh/Core/IO/importer/BaseImporter.hh>
#include <OpenMesh/Core/IO/reader/BaseReader.hh>

// STD C++
#include <iosfwd>
#include <string>


//== NAMESPACES ===============================================================


namespace OpenMesh {
namespace IO {


//== IMPLEMENTATION ===========================================================


/**
    Implementation of the OM format reader. This class is singleton'ed by
    SingletonT to OMReader.
*/
class OPENMESHDLLEXPORT _OM2Reader_ : public BaseReader
{
public:

  _OM2Reader_();
  virtual ~_OM2Reader_() { }

  std::string get_description() const { return "OpenMesh2 File Format"; }
  std::string get_extensions()  const { return "om2"; }
  std::string get_magic()       const { return "OM2"; }

  bool read(const std::string& _filename,
	    BaseImporter& _bi,
	    Options& _opt );

//!  Stream Reader for std::istream input in binary format
  bool read(std::istream& _is,
	    BaseImporter& _bi,
	    Options& _opt );

  virtual bool can_u_read(const std::string& _filename) const;
  virtual bool can_u_read(std::istream& _is) const;

  bool read_ascii(std::istream& _is, BaseImporter& _bi, Options& _opt) const;
  bool read_binary(std::istream& _is, BaseImporter& _bi, Options& _opt) const;
private:
};


//== TYPE DEFINITION ==========================================================


/// Declare the single entity of the OM reader.
extern _OM2Reader_  __OM2ReaderInstance;
OPENMESHDLLEXPORT _OM2Reader_&  OM2Reader();


//=============================================================================
} // namespace IO
} // namespace OpenMesh
//=============================================================================
#endif
//=============================================================================
