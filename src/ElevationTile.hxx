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
#include <iostream>
#include <boost/format.hpp>
#include "Point.hxx"
#include "SaveRestObj.hxx"

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
  class BoundingBox : public SaveRestoreObj {
  public:
    BoundingBox(const Point & sw, const Point & ne) {
      setBoundingBox(sw, ne); 
    }

    // allow key lookup.
    BoundingBox(const Point & pt) {
      setBoundingBox(pt, pt); 
    }

    BoundingBox() {
      sw_lat = sw_lon = ne_lat = ne_lon = 0.0; 
    }

    void setBoundingBox(const Point & sw, const Point & ne);

    bool operator<(const BoundingBox & other) const;

    /**
     * @brief Given a point in a bounding box, and a number of steps in 
     * each direction, calculate the index into an array of points over the
     * bounding box, and the offset from the southwest corner. 
     * 
     * @param pt the point of interest
     * @param lat_pts the number of latitude points in a column of the elevation array
     * @param lon_pts the number of longitude points in a row of the elevation array
     * @param lat_idx (output) the column index into the two dimensional elevation array
     *                         such that pt within the smallest cell in the array
     *                         that contains the point. 
     * @param lon_idx (output) the row index into the elevation array
     * @param lat_offset (output) the distance in the latitude dimension from the 
     *                         southwest corner of the bounding cell
     * @param lon_offset (output) the distance in the longitude dimension from the 
     *                         southwest corner of the bounding cell
     * 
     * @return true if the point is within the bounding box, false otherwise. 
     * 
     */
    bool getPosition(const Point & pt, unsigned int lat_pts, unsigned int lon_pts, 
		     unsigned int & lat_idx, unsigned int & lon_idx, 
		     double & lat_offset, double & lon_offset) const;
    /**
     * @brief is the point within this bounding box? 
     * 
     * @param pt The whole point of the thing.
     * @return true if the point is within the bounding box, false otherwise. 
     */
    bool isIn(const Point & pt) const;

    Point getSW() const {
      return Point(sw_lat, sw_lon); 
    }
    
    Point getNE() const {
      return Point(ne_lat, ne_lon);
    }

    void save(std::ostream & os) {
      os.write((char*)&sw_lat, sizeof(double));
      os.write((char*)&sw_lon, sizeof(double));
      os.write((char*)&ne_lat, sizeof(double));
      os.write((char*)&ne_lon, sizeof(double));      
    }

    void restore(std::istream & is) {
      is.read((char*)&sw_lat, sizeof(sw_lat));
      is.read((char*)&sw_lon, sizeof(sw_lon));      
      is.read((char*)&ne_lat, sizeof(sw_lat));
      is.read((char*)&ne_lon, sizeof(ne_lon));      
    }
    
  private: 
    double sw_lat, sw_lon, ne_lat, ne_lon; 
  };
  
  class ElevationTile : public SaveRestoreObj {
  public:      
    /**
     * @brief empty constructor -- useful for later calls to restore. 
     */
    ElevationTile() {
    }

    void setParams(const Point & sw, const Point & ne) {
      bbox = BoundingBox(sw, ne);
    }

    /**
     * @brief prepare the tile for queries and such. 
     */
    virtual void prepare() { }
    
    /**
     * @brief Is the supplied point inside the region covered by this tile? 
     * 
     * @param loc location to be tested
     * @return true if the point is within this tile
     */
    bool isIn(const Point & loc) {
      BoundingBox ptbb(loc); 
      return !(ptbb < bbox) && !(bbox < ptbb);
    }

    /**
     * @brief Given a point, find it in this tile and return the elevation
     * 
     * @param point where are we looking? 
     * @param elev (output) the point's elevation. 
     * @return true if we found the point in this tile. If false,
     * ignore the value of elev
     */
    virtual bool getElevation(const Point & point, short & elev) const = 0;

    
    const BoundingBox & getBoundingBox() const { return bbox; }

  protected:
    BoundingBox bbox; 
  }; 

}
#endif
