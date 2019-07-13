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
#include <map>
#include <list>
#include "Point.hxx"
#include "Path.hxx"
#include "ElevationTile.hxx"
#include <type_traits>
#include <utility>
#include <boost/format.hpp>

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
  
  class ElevationDB_Base {
  public:
    ElevationDB_Base() {
    }

    
  };
  
  template<typename TileClass> class ElevationDB : public ElevationDB_Base {
    // Test to ensure TileClass is a subclass of ElevationTile...
    typedef typename std::enable_if<std::is_base_of<ElevationTile, TileClass>::value>::type     TileClass_must_be_derived_from_ElevationTile; 

  public:      
    /**
     * @brief Create the elevation database from a set of tiles
     * 
     * @param file_name_list list of files to be read, one for each tile.
     * 
     */
    ElevationDB(const std::list<std::string> & file_name_list) {
      for(auto & fn: file_name_list) {
	registerTile(new TileClass(fn));
      }
    }; 

    /**
     * @brief Create the elevation database from a stream producing a list of filenames
     * 
     * @param lstr the stream with the list of filenames
     * 
     */
    ElevationDB(std::istream & lstr) {
      std::string tile_file_name;
      while(lstr >> tile_file_name) {
	registerTile(new TileClass(tile_file_name));
      }
    }


    
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
    void registerTile(TileClass * tile_p) {
      // do any "cleanup" or preparation that the tile object might need. 
      tile_p->prepare();

      BoundingBox bb = tile_p->getBoundingBox(); 
      // std::cerr << boost::format("Bounding box SW = %s  NE = %s\n")
      // 	% bb.getSW().toString() % bb.getNE().toString();
      
      tile_map[bb] = tile_p;
      tile_list.push_back(tile_p);

      // check the integrity of the list
      if(tile_list.size() != tile_map.size()) {
	// std::cerr << "tile_list and tile_map size disagree\n";
      }
    }

    TileClass * findTile(const Point & point) {
      BoundingBox bb(point);

      for(auto & me : tile_map) {
	if(me.first.isIn(point)) return me.second;
      }
      // for(auto & me : tile_list) {	
      // 	if(me->getBoundingBox().isIn(point)) return me; 
      // }
      //      return (retp == tile_map.end()) ? NULL : retp->second;
      //      return (tile_map.find(bb) != tile_map.end()) ? tile_map[bb] : NULL;
      return NULL;
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
      if(tile == NULL) {
	// std::cerr << boost::format("Could not find a tile for point %s\n")
	//   % point.toString();
	return false; 
      }
      
      // std::cerr << boost::format("Found a tile for point %s\n")
      // 	% point.toString();
      // std::cerr << boost::format("Tile Bounding box SW = %s  NE = %s\n")
      // 	% tile->getBoundingBox().getSW().toString() % tile->getBoundingBox().getNE().toString();      
      
      return tile->getElevation(point, elev);

      return true; 
    }
    
  private:
    std::map<BoundingBox, TileClass *> tile_map;
    std::list<TileClass *> tile_list;    
  }; 

}
#endif
