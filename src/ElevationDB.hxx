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
#include <iostream>
#include <fstream>
#include "Point.hxx"
#include "Path.hxx"
#include "ElevationTile.hxx"
#include "SaveRestObj.hxx"
#include <type_traits>
#include <utility>
#include <boost/format.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/device/file_descriptor.hpp>
#include <boost/filesystem.hpp>
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
  
  class ElevationDB_Base : public SaveRestoreObj {
  public:
    ElevationDB_Base() {
    }

    /**
     * @brief create a compressed database from a stream producing a list of filenames
     */
    void makeDB(std::istream & lstr, 
		const std::string & sav_name, 
		const std::string & map_typename);


    virtual ElevationTile * makeTile(const std::string & fname) = 0;
    virtual ElevationTile * makeTile() = 0;
    void registerTile(ElevationTile * tile_p);

    /**
     * @brief Given a point, find its enclosing tile and return the elevation
     * 
     * @param point where are we looking? 
     * @param elev (output) the point's elevation. 
     * @return true if we found the point on the map. If false,
     * ignore the value of elev
     */
    bool getElevation(const Point & point, short & elev);

    /**
     * @brief Given a path, lookup each point in the elevation table
     * and update its elevation member. 
     * 
     * @param path sequence of points from start to finish
     * @param elevations (output) vector of elevations, one element per point in the path
     */
    void scanPath(Path & path, std::vector<short> & elevations);

    virtual void deleteTile(ElevationTile * tile_p) = 0;
    
    void restore(const std::string & fname);

    void restore(std::istream & is);

    void save(const std::string & fname);

    void save(std::ostream & os);    
    
    
    ElevationTile * findTile(const Point & point);
    
    bool findHighPoint(const Point & point, 
			Point & high_point, 
		       double radius = 0.5, double step = 0.1);
    
  protected:
    std::map<BoundingBox, ElevationTile *> tile_map;    
  };
  
  template<typename TileClass> class ElevationDB : public ElevationDB_Base {
    // Test to ensure TileClass is a subclass of ElevationTile...
    typedef typename std::enable_if<std::is_base_of<ElevationTile, TileClass>::value>::type     TileClass_must_be_derived_from_ElevationTile; 

  public:      
    /**
     * @brief Create an empty elevation database (for a later call to RESTORE)
     * 
     */
    ElevationDB() {
    }; 

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
     * @brief Create a TileClass object for use in ElevationDB_Base
     */
    ElevationTile * makeTile(const std::string & fname) {
      return new TileClass(fname);
    }

    ElevationTile * makeTile() {
      return new TileClass();
    }

    void deleteTile(ElevationTile * tile_p) {
      delete dynamic_cast<TileClass*>(tile_p);
    }

  private:
  }; 

}
#endif
