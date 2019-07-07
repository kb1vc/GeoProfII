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
#ifndef PATH_HDR_DEF
#define PATH_HDR_DEF
#include <string>
#include <regex>
#include <cmath>
#include <vector>

#include "Point.hxx"

/**
 * \class GeoProf::Path
 *
 * \brief Path connects two GeoProf::Points via a great circle route.  
 * It does this by "walking" from the start to the end in steps
 * of a specified length.  The bearing to the endpoint is calculated
 * at each intermediate point. 
 *
 * This class is a subclass of std::vector<Point> to provide useful
 * iterators. 
 * 
 * \author $Author: kb1vc $
 *
 * \date $Date: 2005/04/14 14:16:20 $
 *
 * Contact: kb1vc@kb1vc.org
 *
 */
namespace GeoProf {
  class Path : public std::vector<GeoProf::Point> {
  public:      
    /**
     * @brief Create a great circle path from a start point to an end point. 
     * 
     * @param from the starting point
     * @param to the ending point
     * @param step the distance between each point in the path. 
     */
    Path(Point & from, Point & to, double step) {
      createPath(from, to, step); 
    }

    /**
     * @brief Initialize and create a path from a start to an end point. 
     * Clear out the current path if necessary. 
     *
     * @param from the starting point
     * @param to the ending point
     * @param step the distance between each point in the path. 
     */
    void createPath(Point & from, Point & to, double step);
  }; 

}
#endif
