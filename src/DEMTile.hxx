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
#ifndef DEM_TILE_HDR_DEF
#define DEM_TILE_HDR_DEF
#include <string>
#include <regex>
#include <cmath>
#include <vector>

#include "ElevationTile.hxx"


/**
 * \class GeoProf::DEMTile
 *
 * \brief DEMTile is a specialization of ElevationTile that is 
 * built from the contents of a USGS 3 arc-second elevation file. 
 * 
 * This class is distinct from the DEM class as the latter is 
 * a reader that will be discarded once the DEM file is parsed.  
 * It is meant that the DEMTile will be pickle-able
 *
 * \author $Author: kb1vc $
 *
 * \date $Date: 2005/04/14 14:16:20 $
 *
 * Contact: kb1vc@kb1vc.org
 *
 */

#include "ElevationTile.hxx"
#include "DEM.hxx"

namespace GeoProf {
  class DEMTile : public ElevationTile {
  public:      
    /**
     * @brief Initialize the tables
     * 
     */
    DEMTile(const std::string & fname) {
      DEM(fname, this);
    }

    void setParams(const Point & sw, const Point & ne, 
		   unsigned int _cols) {
      ElevationTile::setParams(sw, ne);
      rows = 0;
      cols = _cols; 
      elevation_array.resize(cols);
    }
    
    std::vector<double> & getProfile(unsigned int col) { 
      // remember columns are numbered starting with "1"
      return elevation_array[col-1]; 
    }
    
    /**
     * @brief Given a point, find it in this tile and return the elevation
     * 
     * @param point where are we looking? 
     * @param elev (output) the point's elevation. 
     * @return true if we found the point in this tile. If false,
     * ignore the value of elev
     */
    bool getElevation(const Point & point, double & elev) const;

    /**
     * @brief prepare the tile for queries and such. 
     */
    void prepare() {
      rows = 0; 
      for(auto & prof : elevation_array) {
	unsigned int sz = prof.size();
	rows = (rows < sz) ? sz : rows; 
      }
    }
    
  private:

    std::vector<std::vector< double >> elevation_array;

    unsigned int rows, cols; 

    bool pointToIndex(const Point & point, unsigned int & x, unsigned int & y); 
  }; 

}
#endif
