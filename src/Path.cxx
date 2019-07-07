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
#include "Path.hxx"
#include <boost/format.hpp>
#include <regex>
#include <stdexcept>
#include <iostream>
#include <cmath>
#include <math.h>
#include <limits>

namespace GeoProf {
  void Path::createPath(Point & from, Point & to, double step) {
    if(this->size() > 0) this->clear();

    // now estimate the total run.
    double bearing, dmy, distance; 
    from.bearingDistanceTo(to, bearing, dmy, distance);

    from.correctBearingDistanceTo(to, bearing, distance, bearing, distance);
    
    // size this vector to be pretty close to what we need.
    unsigned long len = static_cast<unsigned long>(100 + distance / step);
    
    // allocate enough space to get us most of the way there. 
    this->reserve(len); 

    // register the first point
    this->push_back(from);
    
    // now iterate from the start to the end.
    // We know to stop when our distance to the "to" point is less
    // than step.
    Point current = from; 
    Point next;    
    int count = 0; 
    std::cerr << boost::format("From: [%f %f]  To: [%f %f] Bearing %f Distance %f\n")
      % from.getLatitude() % from.getLongitude()
      % to.getLatitude() % to.getLongitude()
      % bearing
      % distance;
    
    for(double path_len = step; path_len < distance; path_len += step) {
      from.stepTo(bearing, path_len, next); 
      this->push_back(next);
    }

    // register the last point
    this->push_back(to); 
  }
}
