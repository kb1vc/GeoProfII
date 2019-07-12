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
#ifndef ELEV_TILE_HDR_DEF
#define ELEV_TILE_HDR_DEF
#include <string>
#include <regex>
#include <cmath>
#include <vector>

#include "Point.hxx"


/**
 * \class GeoProf::ElevationTile
 *
 * \brief ElevationTile is a base class that holds a rectangular region
 * of an elevation map. 
 * 
 * \author $Author: kb1vc $
 *
 * \date $Date: 2005/04/14 14:16:20 $
 *
 * Contact: kb1vc@kb1vc.org
 *
 */
namespace GeoProf {
  class ElevationTile {
  public:      
    /**
     * @brief Specify the region for this elevation tile. 
     * The derived class is responsible for all other initialization
     *
     * @param sw the southwest corner of this tile
     * @param ne the northeast corner of this tile
     */
    ElevationTile(const Point & sw, const Point & ne) {
      sw_lat = sw.getLatitude();
      sw_lon = sw.getLongitude();
      ne_lat = ne.getLatitude();
      ne_lon = ne.getLongitude();
    }

    ElevationTile() {
    }

    /**
     * @brief A comparison function 
     * 
     * @param a the first tile we're comparing
     * @param b the other tile we're comparing
     * 
     * @return 1 if a is to the east of b (the hell with the dateline)
     * and 0 if a and b overlap
     * and -1 if a is to the west of b (the hell with the dateline)
     */
    static int compareLon(const ElevationTile & a, const ElevationTile & b) {
      if (a.sw_lon > b.ne_lon) return 1;
      else if (a.ne_lon < b.sw_lon) return -1;       
      else return 0;
    }

    /**
     * @brief A comparison function 
     * 
     * @param a the first tile we're comparing
     * @param b the other tile we're comparing
     * 
     * @return 1 if a is to the north of b
     * and 0 if a and b overlap
     * and -1 if a is to the south of b
     */
    static bool isNorth(const ElevationTile & a, const ElevationTile & b) {
      if (a.sw_lat > b.ne_lat) return 1;
      else if (a.ne_lat < b.sw_lat) return -1;       
      else return 0;
    }

    /**
     * @brief Is the supplied point inside the region covered by this tile? 
     * 
     * @param loc location to be tested
     * @return the location of the point relative to this tile
     */
    enum RelativeLocation { EAST, WEST, NORTH, SOUTH, INSIDE };
    RelativeLocation isIn(const Point & loc) {
      double loc_lat = loc.getLatitude();
      double loc_lon = loc.getLongitude();      
      if(loc_lat > ne_lat) return NORTH;
      if(loc_lat < sw_lat) return SOUTH;
      if(loc_lon > ne_lon) return EAST;
      if(loc_lon < sw_lon) return WEST;
      // we must be in the tile.
      return INSIDE; 
    }

    /**
     * @brief Given a point, find it in this tile and return the elevation
     * 
     * @param point where are we looking? 
     * @param elev (output) the point's elevation. 
     * @return true if we found the point in this tile. If false,
     * ignore the value of elev
     */
    virtual bool getElevation(const Point & point, double & elev) const = 0;

    /**
     * @brief save this elevation table to a binary stream
     * 
     * @param os the output stream
     */
    virtual void save(std::ostream & os) const { }

    /** 
     * @brief build this elevation table from a binary stream
     * @param ins the input stream
     */
    virtual void restore(std::istream & ins) { };

    
  protected:
    double sw_lat, sw_lon, ne_lat, ne_lon; 
  }; 

}
#endif
