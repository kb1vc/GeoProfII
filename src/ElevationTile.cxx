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
#include "ElevationTile.hxx"


void GeoProf::BoundingBox::setBoundingBox(const Point & sw, const Point & ne) {
  sw_lat = sw.getLatitude();
  sw_lon = sw.getLongitude();
  ne_lat = ne.getLatitude();
  ne_lon = ne.getLongitude();
}

bool GeoProf::BoundingBox::operator<(const BoundingBox & other) const {
  bool retval; 

  double this_comp_loc = 360.0 * 0.5 * (ne_lat + sw_lat) + 0.5 * (ne_lon + sw_lon);
  double other_comp_loc = 360.0 * 0.5 * (other.ne_lat + other.sw_lat) + 0.5 * (other.ne_lon + other.sw_lon);
  if(other.sw_lat == other.ne_lat) { // we're looking at a point
    if(isIn(Point(other.sw_lat, other.sw_lon))) {
      // std::cerr << "other point is in bounding box\n";
      retval = false;
    }
    else {
      // std::cerr << "BIng\n";
      if(sw_lat <= other.sw_lat) {
	// point is below the tile
	// std::cerr << "\t A\n";
	retval = true;
      }
      else if(sw_lat == ne_lat) {
	// point is in the same row as the tile
	// is it to the west of the tile?
	// std::cerr << "\t B\n";	
	retval = sw_lon <= other.sw_lon;
      }
      else {
	// point is above the tile
	// std::cerr << "\t C\n";
	retval = false; 
      }
    }
  }
  else if(sw_lat == ne_lat) { // we are a point
    if(other.isIn(Point(sw_lat, sw_lon))) {
      // std::cerr << "this point %s is in bounding box\n";
      retval = false;
    }
    else {
      // std::cerr << "Bang\n";      
      retval = (other.sw_lat < sw_lat) ?  (other.sw_lon < sw_lon) : false;
    }
  }
  else if(this_comp_loc > other_comp_loc) {
    // std::cerr << "Bong\n"; 
    retval = false;
  }
  else {
    // std::cerr << "Beep\n";
    retval = true; 
  }

  // if(retval && (other < *this)) {
  //   std::cerr << boost::format("OOOOPS!  Comparing [%g %g -- %g %g] to [%g %g -- %g %g] got %c got TRUE both ways\n")
  //     % sw_lat % sw_lon % ne_lat % ne_lon 
  //     % other.sw_lat % other.sw_lon % other.ne_lat % other.ne_lon 
  //     % ((char) (retval ? 'T' : 'F'));
  // }
  // std::cerr << boost::format("Comparing [%g %g -- %g %g] (%g) to [%g %g -- %g %g] (%g) got %c\n\n\n")
  //   % sw_lat % sw_lon % ne_lat % ne_lon % this_comp_loc
  //   % other.sw_lat % other.sw_lon % other.ne_lat % other.ne_lon % other_comp_loc
  //   % ((char) (retval ? 'T' : 'F'));

  return retval; 
}

bool GeoProf::BoundingBox::getPosition(const Point & pt, unsigned int lat_pts, unsigned int lon_pts, 
				       unsigned int & lat_idx, unsigned int & lon_idx, 
				       double & lat_offset, double & lon_offset) const {
  if(!isIn(pt)) return false;

  double p_lat = pt.getLatitude();
  double p_lon = pt.getLongitude();

  // note there are lat_pts - 1 intervals in the col dimension
  // and lon_pts - 1 intervals in the row dimension.
  double lat_deg_p_pt = (ne_lat - sw_lat) / lat_pts;
  double d_lat_idx = floor((p_lat - sw_lat) / lat_deg_p_pt);
  lat_idx = ((unsigned int) d_lat_idx);
  lat_offset = p_lat - d_lat_idx * lat_deg_p_pt; 

  double lon_deg_p_pt = (ne_lon - sw_lon) / lon_pts;
  double d_lon_idx = floor((p_lon - sw_lon) / lon_deg_p_pt);
  lon_idx = ((unsigned int) d_lon_idx);
  lon_offset = p_lon - d_lon_idx * lon_deg_p_pt; 
      
  // std::cerr << boost::format("getOffset for pt %s lat_idx = %d lon_idx = %d  lat_pts %d lon_pts %d lat_deg_p_pt = %g  lon_deg_p_pt %g\n")
  //   % pt.toString() % lat_idx % lon_idx % lat_pts % lon_pts % lat_deg_p_pt % lon_deg_p_pt; 
  return true; 
}

/**
 * @brief is the point within this bounding box? 
 * 
 * @param pt The whole point of the thing.
 * @return true if the point is within the bounding box, false otherwise. 
 */
bool GeoProf::BoundingBox::isIn(const Point & pt) const {
  double p_lat = pt.getLatitude();
  double p_lon = pt.getLongitude();
  bool retval = (p_lat >= sw_lat) && (p_lat <= ne_lat) &&
    (p_lon >= sw_lon) && (p_lon <= ne_lon);

  // std::cerr << boost::format("isIn is testing pt %s against sw [%g %g] ne [%g %g] -> %c\n")
  //   % pt.toString() % sw_lat % sw_lon % ne_lat % ne_lon 
  //   % ((char) (retval ? 'T' : 'F'));

  return retval; 
}
