/*
  Copyright (c) 2019 Matthew H. Reilly (kb1vc)
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in
  the documentation and/or other materials provided with the
  distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "ZFile.hxx"
#include <boost/format.hpp>
#include <stdexcept>
#include <iostream>

namespace GeoProf {
  ZFile::ZFile(const std::string & fname, unsigned int _max_retry_count) {
    max_retry_count = _max_retry_count; 

    fdes = gzopen(fname.c_str(), "rb");

    if(fdes == NULL) throw std::runtime_error((boost::format("GeoProf::ZFile could not open file \"%s\"\n") % fname).str());
  }

  bool ZFile::read(char * obj_ptr, size_t len) {
    char * cur_ptr = obj_ptr;
    size_t left = len; 

    unsigned int retry_count = 0; 

    while(left > 0) {
      size_t rsize = gzread(fdes, cur_ptr, left);
      if(rsize == 0) return -1;
      else if((rsize < 0) && (errno == EAGAIN)) {
	retry_count++; 
	if(retry_count > max_retry_count) return false; 
      }
      else if(rsize < 0) {
	return false; 
      }
      else {
	left -= rsize;
	cur_ptr += rsize; 
      }
    }

    return true;
  }

}
