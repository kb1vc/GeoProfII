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
#ifndef DEM_HDR_DEF
#define DEM_HDR_DEF
#include <string>
#include <regex>
#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/format.hpp>
#include <regex>

#include "Point.hxx"
#include "ZFile.hxx"

/**
 * \class GeoProf::DEM
 *
 * \brief DEM is a specialization of the ElevationDB class that
 * maintains an elevation database read from binary elevation format
 * files (used in the original GeoProf from 1996). 
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
  
  class DEM {
  public:      
    /**
     * @brief Load a binary elevation table
     * 
     * @param fname names a binary elevation file. 
     */
    DEM(const std::string & fname) {
      inf.open(fname);

      // read the header.   See "Standards for Digital Elevation Models -- Part2 : Specifications -- Appendix 2-A
      readHeader();
    }

    std::string file_name;
    std::string description;
    Point corners[4];

    
  private:

    static std::regex float_cleanup_re;
    
    void readHeader();

    void cleanupFloat(std::string & str) {
      str = std::regex_replace(str, float_cleanup_re, "$1 ");
    }

    template<typename T> void extract(std::string & str, 
				      unsigned int first_col, 
				      unsigned int last_col, 
				      T & ret) {
      
      std::string sub = str.substr(first_col - 1, 1 + last_col - first_col);
      cleanupFloat(sub);

      std::stringstream strstr(sub);
      strstr >> ret; 
    }
      
    template<typename T> void extractVector(std::string & str, 
					    unsigned int first_col, 
					    unsigned int last_col, 
					    std::vector<T> & vec, 
					    unsigned int veclen) {
      std::string sub = str.substr(first_col - 1, 1 + last_col - first_col);
      cleanupFloat(sub);
      std::stringstream strstr(sub);
      for(unsigned int i = 0; i < veclen; i++) {
	T val; 
	strstr >> val;
	vec.push_back(val);
      }
    }

    std::ifstream inf; 
  }; 
}
#endif
