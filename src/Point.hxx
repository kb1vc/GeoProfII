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
#ifndef POINT_HDR_DEF
#define POINT_HDR_DEF
#include <string>
#include <regex>
#include <cmath>
#include <boost/format.hpp>

/**
 * \class GeoProf::Point
 *
 * \brief Point implements navigation operations on a geographic point
 * on the surface of the earth. The Point object includes methods to 
 * calculate the distance and bearing from one point to another, or the
 * location (point) at a given distance and bearing from a starting location.
 *
 * \author $Author: kb1vc $
 *
 * \date $Date: 2005/04/14 14:16:20 $
 *
 * Contact: kb1vc@kb1vc.org
 *
 */
namespace GeoProf {
  class Point {
  public:      
    /**
     * @brief Create a point object from latitude, and longitude
     * 
     * @param _lat the latitude of the point -- negative values are south of the equator
     * @param _lon the longitude of the point -- negative values are west of Grenwich
     * 
     * Note that bearings between the poles are unlikely to make sense.
     * In places where every direction is "south" or "north" the math
     * is often beyond sensible limits.  If your application requires
     * pole-to-pole paths, then look elsewhere, or just point south.
     */
    Point(double _lat = 0.0, double _lon = 0.0) {
      lat = _lat;
      lon = _lon;
    }

    /**
     * @brief Create a point object from another point object
     *
     * @param orig the point that we're cloning. 
     */
    Point(const Point & orig) {
      lat = orig.lat;
      lon = orig.lon;
    }

    /**
     * @brief Create a point from a Maidenhead Grid specifier
     * 
     * @param grid A Maidenhead Grid specifier of the form XXnnxx (capital
     * letter pair, digit pair, letter pair)
     */
    Point(const std::string & grid) {
      grid2Pt(grid);
    }


    /**
     * @brief Return degrees latitude (negative is south of the equator)
     * 
     * @return latitude
     */
    double getLatitude() const { return lat; }

    /**
     * @brief Return degrees longitude (negative is west of the Grenwich)
     * 
     * @return longitude
     */
    double getLongitude() const { return lon; }

    
    /**
     * @brief Calculate the bearing (in degrees -- 0 is north) from this point to anothe point
     * along the shorter great circle route
     *
     * @param other the other point
     * @return bearing in degrees.
     */
    double bearingTo(const Point & other) const;

    
    /**
     * @brief Calculate the great circle distance (in meters) from this point to another point
     *
     * @param other the other point
     * @return great circle distance in meters
     */
    double distanceTo(const Point & other) const;

    /**
     * @brief Calculate the bearing (in degrees -- 0 is north) and distance (in meters) 
     * from this point to another point
     * along the shorter great circle route
     *
     * @param other the other point
     * @param bearing (output) direction to the other point along the shorter great circle path
     * @param reverse_bearing direction of travel from other point to this point (in degrees, 0 is north)
     * @param distance (output) distance in meters to the other point along the shorter great circle path
     */
    void bearingDistanceTo(const Point & other, double & bearing, double & reverse_bearing, double & distance) const;

    /**
     * @brief Return the point at which one would arrive after traveling on the 
     * great circle path from this point at the specified bearing and for the specified distance.
     *
     * @param bearing direction of travel (in degrees, 0 is north)
     * @param distance of travel in meters
     * @param next (output) the point at which we'll arrive.
     */
    void stepTo(double bearing, double distance, Point & next) const;

    /**
     * @brief convert this point to its Maidenhead Grid specifier
     * 
     * @param grid (output) The Maidenhead Grid string XXnnxx
     */
    void pt2Grid(std::string & grid) const;

    /**
     * @brief Set this point to the location of a Maidenhead Grid specifier
     * 
     * @param grid A Maidenhead Grid specifier of the form XXnnxx (capital
     * letter pair, digit pair, letter pair)
     */
    void grid2Pt(const std::string & grid);

    /**
     * @brief Set this point from a string describing the location in 
     * degrees-minutes-seconds of latitude and logitude. Specification is
     * in lat degrees/minutes/seconds and lon degrees/minutes/seconds as 
     * doubles.  North/South/East/West 
     * 
     * @param lat_d latitude degrees
     * @param lat_m latitude minutes
     * @param lat_s latitude seconds
     * @param ns true if latitude is north of the equator
     * @param lon_d longitude degrees
     * @param lon_m longitude minutes
     * @param lon_s longitude seconds
     * @param ew true if latitude is east of Grenwich
     */
    void dms2Pt(double lat_d, double lat_m, double lat_s, char ns,
		 double lon_d, double lon_m, double lon_s, char ew);

    /**
     * @brief Correct the calculated bearing and distance by iterating
     * around the calculated values to find the minimum error. 
     *
     * @param other the other point
     * @param bearing direction to the other point along the shorter great circle path
     * @param distance distance in meters to the other point along the shorter great circle path
     * @param new_bearing (output) corrected bearing
     * @param new_distance (output) corrected range
     * 
     */
    void correctBearingDistanceTo(const Point & other, 
				  const double bearing, 
				  const double distance, 
				  double & new_bearing, 
				  double & new_distance) const;


    std::string toString() const {
      char ns = ' ';
      if (lat > 0.) {
	ns = 'N';
      }
      if (lat < 0.0) {
	ns = 'S';
      }

      char ew = ' ';
      if (lon > 0.0) {
	ew = 'E';
      }
      if (lon < 0.0) {
	ew = 'W';
      }
      
      return (boost::format("%g %c %g %c") % lat % ns % lon % ew).str();
    }
  private:
    /// Latitude South is negative, North is positive. 
    double lat;
    
    /// Longitude West is negative, East is positive. 
    double lon;

    static std::regex grid_regexp; //  ("[A-R][A-R][0-9][0-9][A-X][A-X]", std::regex_constants::icase);      
    
    /**
     * @brief Validate a string as a Maidenhead Grid specifier
     *
     * @param grid A Maidenhead Grid specifier of the form XXnnxx (capital
     * letter pair, digit pair, letter pair)
     * @return true if this is a properly formatted grid, false otherwise. 
     */
    bool checkGrid(const std::string & grid) const;
    
    /**
     * @brief Helper for translating char positions in a grid 
     * locator into double offsets from 0 degrees.
     */
    double gridDiff(char v, char s, double mul) const;

    /**
     * @brief four quadrant arc tangent returning result in the range 0..2pi
     * 
     * @param y rise
     * @param x run
     * @return arctan in range 0..2pi
     */
    double atan2Pt(double y, double x) const;

    /**
     * @brief translate an angle into the range 0..2pi
     * 
     * @param ang angle in radians
     * @return angle in range 0..2pi
     */
    double inSpan(double ang) const;

    // recursive helper to correctBearingDistanceTo
    bool recCorrectBearingDistanceTo(const Point & other, 
				     const double bearing, 
				     const double distance, 
				     const double b_span,
				     const double d_span,
				     double & new_bearing, 
				     double & new_distance
				     ) const;
    
    // Useful constants
    static const double clarke_66_al;    /*Clarke 1866 ellipsoid*/
    static const double clarke_66_bl;
    static const double rad_per_deg;
  }; 

}
#endif
