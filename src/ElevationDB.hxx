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
#ifndef ELEVDB_HDR_DEF
#define ELEVDB_HDR_DEF
#include <string>
#include <regex>
#include <cmath>
#include <vector>

#include "Point.hxx"
#include "Path.hxx"
#include "ElevationTile.hxx"
#include <type_traits>
#include <utility>

/**
 * \class GeoProf::ElevationDB
 *
 * \brief ElevationDB contains a list or map of ElevationTiles. 
 * The tile table allows rapid lookup of a point to find the enclosing
 * tile. 
 *
 * It is a template class. The TileClass supplied to the template 
 * must be a subclass of ElevationTile. 
 * 
 * \author $Author: kb1vc $
 *
 * \date $Date: 2005/04/14 14:16:20 $
 *
 * Contact: kb1vc@kb1vc.org
 *
 */
namespace GeoProf {
  
  template<typename TileClass> class ElevationDB {
    // Test to ensure TileClass is a subclass of ElevationTile...
    typedef typename std::enable_if<std::is_base_of<ElevationTile, TileClass>::value>::type     TileClass_must_be_derived_from_ElevationTile; 

  public:      
    /**
     * @brief Create the elevation database -- this is empty. 
     * 
     */
    ElevationDB() {
      needs_sorting = true; 
    }; 

    /**
     * @brief Given a path, lookup each point in the elevation table
     * and update its elevation member. 
     * 
     * @param path sequence of points from start to finish
     */
    void scanPath(Path & path); 

    /** 
     * @brief register a new tile
     */
    void registerTile(TileClass * tile) {
      needs_sorting = true; 
    }

    void sortTiles() {
      // sort the tiles by corners. 
    }

    TileClass * findTile(const Point & point) {
      if(needs_sorting) sortTiles();

      // now scan the vector from the middle on out. 
      int hlat_len = tile_array.size() >> 1;
      int hlon_len = tile_array[hlat_len].size() >> 1;
	
      recursiveFindTile(point, hlat_len / 2, hlon_len / 2, 
			0, hlat_len - 1, 0, hlon_len - 1); 
    }

    /**
     * @brief Given a point, find its enclosing tile and return the elevation
     * 
     * @param point where are we looking? 
     * @param elev (output) the point's elevation. 
     * @return true if we found the point on the map. If false,
     * ignore the value of elev
     */
    double getElevation(const Point & point, double & elev) {
      TileClass * tile = findTile(point); 
      if(tile == NULL) return false; 
      
      return tile->getElevation(point, elev);

      return true; 
    }
    
  private:
    
    TileClass * recursiveFindTile(const Point & point, 
				      int lat_idx, int lon_idx,
				      int lat_idx_lo, int lat_idx_hi, 				      
				      int lon_idx_lo, int lon_idx_hi) {
      TileClass * cur_tile = tile_array[lat_idx][lon_idx];
      ElevationTile::RelativeLocation in_res = cur_tile->isIn(point);
      int old_lat_idx = lat_idx;
      int old_lon_idx = lon_idx;       
      switch(in_res) {
      case ElevationTile::NORTH:
	lat_idx_lo = lat_idx; 	
	lat_idx = (lat_idx_hi + lat_idx_lo) / 2; 
	break; 
      case ElevationTile::SOUTH:
	lat_idx_hi = lat_idx; 	
	lat_idx = (lat_idx_hi + lat_idx_lo) / 2; 
	break; 
      case ElevationTile::EAST: 
	lon_idx_lo = lon_idx;
	lon_idx = (lon_idx_hi + lon_idx_lo) / 2; 	
	break; 
      case ElevationTile::WEST: 
	lon_idx_hi = lon_idx;
	lon_idx = (lon_idx_hi + lon_idx_lo) / 2; 	
	break; 
      case ElevationTile::INSIDE:
	return cur_tile;
	break;
      }

      if((lat_idx == old_lat_idx) && (lon_idx == old_lon_idx)) {
	// we came up empty...
	return NULL; 
      }
      return recursiveFindTile(point, lat_idx, lon_idx, 
			       lat_idx_lo, lat_idx_hi, 
			       lon_idx_lo, lon_idx_hi); 
    }

    // Each entry in this list corresponds to a slice of 
    std::vector<std::vector<TileClass *>> tile_array;
    bool needs_sorting; 
    
    // remember the last four tiles that we've found
    std::vector<TileClass *> recent_finds;
  }; 

}
#endif
