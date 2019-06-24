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
#include "Point.hxx"
#include <boost/format.hpp>
#include <regex>
#include <stdexcept>
#include <iostream>
#include <math.h>

namespace GeoProf {

  std::regex Point::grid_regexp("[A-R][A-R][0-9][0-9][A-X][A-X]", std::regex_constants::icase);
  
  double Point::bearingTo(const Point & other) {
    return 0.0;
  }

  double Point::distanceTo(const Point & other) {
    return 0.0;
  }

  void Point::bearingDistanceTo(const Point & other, double & bearing, double & distance) {
  }

  void Point::stepTo(double bearing, double distance, Point & next) {
  }

  void Point::toGrid(std::string & grid) {
    std::string ret(6, ' ');

    // First convert lat and lon to degrees and minutes.
    // Remember the basis is lon 180 is Grenwich, lat 0 is the pole
    // so we need to correct;
    double llon = lon + 180;
    double llat = lat + 90;
    
    int ilat = ((int) floor(llat));
    int ilat_mins = ((int) floor(60.0 * (llat - floor(llat))));

    int ilon = ((int) floor(llon));
    int ilon_mins = ((int) floor(60.0 * (llon - floor(llon))));

    // set lon positions
    ret[0] = 'A' + ((int) (ilon / 20));
    ilon = ilon % 20;
    ret[2] = '0' + (ilon / 2);
    ilon = ilon % 2;
    ret[4] = 'a' + ((ilon * 60 + ilon_mins) / 5);
    
    ret[1] = 'A' + ((int) (ilat / 10));
    ilat = ilat % 10;
    ret[3] = '0' + ilat;
    ret[5] = 'a' + ((ilat_mins * 2) / 5);

    grid = ret; 

    return; 
  }

  void Point::fromStringDMS(const std::string & dms) {
  }


  void Point::fromDMS(double lat_d, double lat_m, double lat_s, char ns,
		      double lon_d, double lon_m, double lon_s, char ew) {
  }

  double Point::gridDiff(char v, char s, double mul) {
    return mul * ((double) (v - s));
  }
  
  void Point::fromGrid(const std::string & grid) {
    if(!checkGrid(grid)) {
      throw std::runtime_error((boost::format("Bad grid specifier: [%s] is not in the proper form.") % grid).str());
    }

    std::string lgrid = grid;

    // upcase all of it... Is anybody embarrassed by the awkwardness here?
    std::transform(lgrid.begin(), lgrid.end(), lgrid.begin(), ::toupper);


    lon = gridDiff(lgrid[0], 'A', 20.0) + 
      gridDiff(lgrid[2], '0', 2.0);

    if(lgrid.size() > 4) {
      lon += (gridDiff(lgrid[4], 'A', 5.0) + 2.5) / 60.0; 
    }
    
    lon = lon - 180.0; 

    
    lat = gridDiff(lgrid[1], 'A', 10.0) + 
      gridDiff(lgrid[3], '0', 1.0);

    if(lgrid.size() > 5) {
      lat += (gridDiff(lgrid[5], 'A', 2.5) + 1.25) / 60.0; 
    }

    lat = lat - 90.0; // correct for south pole being 0deg lat.
  }

  
  bool Point::checkGrid(const std::string & grid)
  {
    /*verifies that the grid square is legitimate*/
    /* return true for a good grid. */

    return regex_match(grid, grid_regexp); 
  }
  
}
