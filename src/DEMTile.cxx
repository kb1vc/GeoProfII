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

bool GeoProf::DEMTile::getElevation(const Point & point, short & elev) const {
  unsigned int lat_idx, lon_idx;
  double lat_offset, lon_offset;
  
  if(bbox.getPosition(point, cols, rows, lat_idx, lon_idx, lat_offset, lon_offset)) {
    // use "at" so that we trigger an exception if this is screwed up. 
    elev = elevation_array.at(lon_idx).at(lat_idx); 
    return true; 
  }
  
  return false; 
}

void GeoProf::DEMTile::restore(std::istream & is) {
  // format is
  // [cols]  
  // [BoundingBox]
  // [rows][profile] ...
  cols = 0; 
  is.read((char*)&cols, sizeof(cols));

  bbox.restore(is);      
  elevation_array.resize(cols);

  for(int i = 0; i < cols; i++) {
    unsigned int rows;
    is.read((char*)&rows, sizeof(rows));
    elevation_array[i].resize(rows);
    //    elevation_array[i].reserve(rows);
    is.read((char*) &(elevation_array[i][0]), rows * sizeof(short));
  }
}

void GeoProf::DEMTile::save(std::ostream & os) {
  // format is
  // [cols]  
  // [BoundingBox]
  // [rows][profile] ...
  cols = elevation_array.size();
  os.write((char*)&cols, sizeof(cols));
  bbox.save(os);       
  for(int i = 0; i < cols; i++) {
    unsigned int rows = elevation_array[i].size();
    os.write(reinterpret_cast<const char *>(&rows), sizeof(rows));
    os.write(reinterpret_cast<const char *>(&elevation_array[i][0]), rows * sizeof(short));
  }
}
