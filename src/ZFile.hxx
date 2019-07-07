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
#ifndef ZFILE_HDR_DEF
#define ZFILE_HDR_DEF
#include <string>
#include <zlib.h>

/**
 * \class GeoProf::ZFile
 *
 * \brief Zfile opens a gzipped file for reading, and provides a 
 * "guaranteed read" operation that will always return a block of
 * the size requested, or will throw an exception. 
 *
 * 
 * \author $Author: kb1vc $
 *
 * \date $Date: 2005/04/14 14:16:20 $
 *
 * Contact: kb1vc@kb1vc.org
 *
 */
namespace GeoProf {
  class ZFile { 
  public:      
    /**
     * @brief Open a gz file for reading. 
     * 
     * @param fname gz file to be read
     * @param _max_retry_count limits the number of times
     *        EAGAIN is returned before we just give up.
     */
    ZFile(const std::string & fname, unsigned int _max_retry_count = 1024);

    /**
     * @brief Read a sequence of bytes from a compressed file
     * 
     * @param obj_ptr pointer to a buffer of char, length len
     * @param len number of bytes in the buffer
     * 
     * @return true if all bytes are read, false otherwise 
     */
    bool read(char * obj_ptr, size_t len);

    /**
     * @brief Read an object from a compressed file. 
     * 
     * @param obj reference to an object to be filled. 
     */
    template<typename T> bool read(T & obj) {
      size_t target_size = sizeof(T);
      char * obj_ptr = reinterpret_cast<char*>(&obj);
      return read(obj_ptr, target_size); 
    }


    
  private:
    gzFile fdes; 

    unsigned int max_retry_count; 

  }; 

}
#endif
