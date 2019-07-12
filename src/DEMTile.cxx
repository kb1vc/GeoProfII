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
#include "DEMTile.hxx"

bool GeoProf::DEMTile::getElevation(const Point & point, double & elev) const {
  return false; 
}
    
bool GeoProf::DEMTile::pointToIndex(const Point & point, unsigned int & x, unsigned int & y) {
  int cols = elevation_array.size();
  // where are we in the range E to W..
  double x_offset = point.getLongitude() - sw_lon;
  double lonrange = ne_lon - sw_lon;
  // cut it up into columns
  double d_xidx = (x_offset / lonrange) * ((double) cols);
  x = (unsigned int) floor(d_xidx);

  if(x >= cols) return false;

  // now find the row
  std::vector<double> & col_vec = elevation_array[x];
  int rows = col_vec.size();
  double y_offset = point.getLatitude() - sw_lat;
  double latrange = ne_lat - sw_lat;
  double d_yidx = (y_offset / latrange) * ((double) rows);
  y = (unsigned int) floor(d_yidx);

  if(y >= rows) return false;

  return true; 
}
