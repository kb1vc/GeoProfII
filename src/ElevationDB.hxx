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
  
  
  template<typename TileClass> class ElevationDB : public SaveRestoreObj {
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
     * @brief create a compressed database from a stream producing a list of filenames
     */
    void makeDB(std::istream & lstr, const std::string & sav_name, const std::string & map_typename) {
      boost::iostreams::filtering_ostream out;
      out.push(boost::iostreams::gzip_compressor(boost::iostreams::gzip_params(boost::iostreams::gzip::best_compression)));
      out.push(boost::iostreams::file_descriptor_sink(sav_name)); 

      std::vector<std::string> fnames; 
      std::string tile_file_name; 
      while(lstr >> tile_file_name) {
	fnames.push_back(tile_file_name); 
      }

      unsigned int num_tiles = fnames.size();
      out.write((char*)&num_tiles, sizeof(num_tiles));

      char typnamebuf[256]; 
      out.write((char*)&typnamebuf, 256);

      int count = 0; 
      for(auto & tfn: fnames) {
	count++; 
	TileClass * tile_p = new TileClass(tfn);
	
	tile_p->save(out);
	
	delete tile_p; 
      }

      boost::iostreams::close(out);
    }
    
    
    /**
     * @brief Given a path, lookup each point in the elevation table
     * and update its elevation member. 
     * 
     * @param path sequence of points from start to finish
     * @param elevations (output) vector of elevations, one element per point in the path
     */
    void scanPath(Path & path, std::vector<short> & elevations) {
      elevations.clear();
      for(auto pt: path) {
	short elev;
	if(getElevation(pt, elev)) {
	  elevations.push_back(elev);	  
	}
	else {
	  elevations.push_back(0);
	}
      }
    }

    /** 
     * @brief register a new tile
     */
    void registerTile(TileClass * tile_p) {
      // do any "cleanup" or preparation that the tile object might need. 
      tile_p->prepare();

      BoundingBox bb = tile_p->getBoundingBox(); 

      tile_map[bb] = tile_p;
    }

    TileClass * findTile(const Point & point) {
      BoundingBox bb(point);

      for(auto & me : tile_map) {
	if(me.first.isIn(point)) return me.second;
      }
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
    bool getElevation(const Point & point, short & elev) {
      TileClass * tile = findTile(point); 
      if(tile == NULL) {
	return false; 
      }
      
      return tile->getElevation(point, elev);

      return true; 
    }

    bool findHighPoint(const Point & point, Point & high_point, 
		       double radius = 0.5, double step = 0.1) {
      short max_el;
      high_point = point; 
      if(!getElevation(point, max_el)) return false; 
      short was_el = max_el;

      // now look everwhere in a 500 meter radius of this point.
      std::vector<short> elevs;
      for(double ang = 0.0; ang < 360.0; ang += 5.0) {
	Point next; 
	point.stepTo(ang, radius, next); 
	GeoProf::Path path(point, next, step);
	scanPath(path, elevs);
	int i = 0; 
	for(auto el: elevs) {
	  if(el > max_el) {
	    high_point = next; 
	    max_el = el; 
	  }
	  i++; 
	}
      }

      return true; 
    }

    void save(const std::string & fname) {
      boost::iostreams::filtering_ostream out;
      out.push(boost::iostreams::gzip_compressor(boost::iostreams::gzip_params(boost::iostreams::gzip::best_compression)));
      out.push(boost::iostreams::file_descriptor_sink(fname)); 

      save(out);

      boost::iostreams::close(out);
    }

    void save(std::ostream & os) {
      // save format is
      // [unsigned int] sizeof_map
      // [char [256] ] tile_type_name
      // one TileClass instance after another.

      unsigned int sizeof_map = tile_map.size();
      char tile_type_name_buf[256];

      os.write((char*)&sizeof_map, sizeof(unsigned int));
      os.write((char*)&tile_type_name_buf, 256); 

      for(auto & tile : tile_map) {
	tile.second->save(os);
      }
    }

    void restore(const std::string fname) {
      std::ifstream compressed_istream(fname, std::ios_base::in | std::ios_base::binary);
      boost::iostreams::filtering_streambuf<boost::iostreams::input> inbuf;          
      inbuf.push(boost::iostreams::gzip_decompressor());
      inbuf.push(compressed_istream);

      std::istream is(&inbuf);

      restore(is); 

      compressed_istream.close();
    }

    void restore(std::istream & is) {
      unsigned int mapsize; 
      is.read((char*)&mapsize, sizeof(mapsize));
      char tile_type_name_buf[256];
      is.read(tile_type_name_buf, 256);

      
      for(int i = 0; i < mapsize; i++) {
	TileClass * tile_p = new TileClass(); 
	tile_p->restore(is);
	registerTile(tile_p);
      }
    }
  private:
    std::map<BoundingBox, TileClass *> tile_map;
  }; 

}
#endif
