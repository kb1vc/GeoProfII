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
#include "Point.hxx"
#define __NO_MATH_INLINES
#include <boost/format.hpp>
#include <regex>
#include <stdexcept>
#include <iostream>
#include <cmath>
#include <math.h>
#include <limits>

namespace GeoProf {

  std::regex Point::grid_regexp("[A-R][A-R][0-9][0-9][A-X][A-X]", std::regex_constants::icase);

    // Useful constants
  const double Point::clarke_66_al = 6378206.4;    /*Clarke 1866 ellipsoid*/
  const double Point::clarke_66_bl = 6356583.8; 
  const double Point::rad_per_deg = (M_PI / 180.0);
  
  double Point::bearingTo(const Point & other) const {
    double br, rbr, dst; 
    bearingDistanceTo(other, br, rbr, dst);
    
    return br; 
  }

  double Point::distanceTo(const Point & other) const {
    double br, rbr, dst; 
    bearingDistanceTo(other, br, rbr, dst);
    
    return dst; 
  }

  // In g++ we get some very bad behavior around generated NANs -- these get handled better with more cautious math.
  __attribute__((optimize("-fno-fast-math")))
  void Point::bearingDistanceTo(const Point & other, double & bearing, double & reverse_bearing, double & distance) const {
    /*Taken directly from:       */
    /*Thomas, P.D., 1970, Spheroidal Geodesics, reference systems,*/
    /*    & local geometry, U.S. Naval Oceanographic Office SP-138,*/
    /*    165 pp.*/

    /*assumes North Latitude and East Longitude are positive*/
    // Translated from C, translated from fortran as in this comment
    /* forward.f -- translated by f2c (version 19960717).
       then hacked up a bit by hand to remove the dependence on 
       the f2c libraries and such. */ 

    double BOA, F, P1R, P2R, L1R, L2R, DLR, T1R, T2R, TM, DTM, STM, CTM, SDTM,
      CDTM, KL, KK, SDLMR, L, CD, DL, SD, T, U, V, D, X, E, Y, A, FF64,
      TDLPM, HAPBR, HAMBR, A1M2, A2M1;

    if((fabs(other.lat - this->lat) < 1.0e-10) && (fabs(other.lon - this->lon) < 1.0e-10)) {
      bearing = 0.0;
      reverse_bearing = 0.0;
      distance = 0.0;
      return;
    }

    BOA = clarke_66_bl / clarke_66_al;
    F = 1.0 - BOA;
    P1R = this->lat * rad_per_deg;
    P2R = other.lat * rad_per_deg;
    L1R = this->lon * rad_per_deg;
    L2R = other.lon * rad_per_deg;
    DLR = L1R - L2R;
    T1R = atan(BOA * tan(P1R));
    T2R = atan(BOA * tan(P2R));
    TM = (T1R + T2R) / 2.0;
    DTM = (T2R - T1R) / 2.0;
    STM = sin(TM);
    CTM = cos(TM);
    SDTM = sin(DTM);
    CDTM = cos(DTM);
    KL = STM * CDTM;
    KK = SDTM * CTM;
    SDLMR = sin(DLR / 2.0);
    L = SDTM * SDTM + SDLMR * SDLMR * (CDTM * CDTM - STM * STM);
    CD = 1.0 - 2.0 * L;
    DL = acos(CD);
    SD = sin(DL);
    T = DL / SD;
    U = 2.0 * KL * KL / (1.0 - L);
    V = 2.0 * KK * KK / L;
    D = 4.0 * T * T;
    X = U + V;
    E = -2.0 * CD;
    Y = U - V;
    A = -D * E;
    FF64 = F * F / 64.0;
    distance = clarke_66_al * SD * (T -
				    F / 4.0 * (T * X - Y) +
				    FF64 * (X * (A + (T - (A + E) / 2.0) * X) +
					    Y * (E * Y - 2.0 * D) + D * X * Y)) / 1000.0;

    double dlrtan = tan(DLR);
    // At times DLR is 2pi... or pi... then we need to
    // fixup the atan calculation, as TDLPM is going to be very very large. 
      
    double tanarg = (DLR - (E * (4.0 - X) + 2.0 * Y)
		     * (F / 2.0 * T + FF64 * (32.0 * T + (A - 20.0 * T) * X - 2.0 * (D + 2.0) * Y)) / 4.0 * dlrtan) / 2.0;

    TDLPM = tan(tanarg);


    HAPBR = atan2Pt(SDTM, CTM * TDLPM); 
    HAMBR = atan2Pt(CDTM, STM * TDLPM); 

    A1M2 = 2.0 * M_PI + HAMBR - HAPBR;
    A2M1 = 2.0 * M_PI - HAMBR - HAPBR;

    // bring A1M2 into the range 0..2pi
    A1M2 = inSpan(A1M2);
    A2M1 = inSpan(A2M1); 

    /* these 360 degree corrections were added to
       fix a disagreement in the semantics of 
       the original implementation of ATAN2, vs
       the implementation from the gnu c math rtl. */
    bearing = (A1M2 == 0.0) ? 0.0 : (360.0d - (A1M2 / rad_per_deg));
    reverse_bearing = (A2M1 == 0.0) ? 0.0 : (360.0d - (A2M1 / rad_per_deg));
    
    if((bearing - 360.0d) >= 0.0) bearing -= 360.0;
    if((reverse_bearing - 360.0d) >= 0.0) reverse_bearing -= 360.0;
  }

  bool Point::recCorrectBearingDistanceTo(const Point & other, 
					  const double bearing, 
					  const double distance, 
					  const double b_span,
					  const double d_span,
					  double & new_bearing, 
					  double & new_distance
					  ) const {
    
    // minimize distance error;
    double err_dist = 1e6; // we're closer than that..
    new_bearing = bearing;
    new_distance = distance; 
    bool ret = false; 
    // sweep +/- 1 degree
    for(double b_inc = -1.0 * b_span; b_inc < b_span; b_inc += (b_span * 0.25)) {
      double az = bearing + b_inc;
      if(az < 0.0) az = az + 360.0;
      if(az > 360.0) az = az - 360.0;
      bool got_better = false; 
      double last_err = 1e6;
      for(double distinc = -1.0 * d_span; distinc < d_span; distinc += (d_span * 0.25)) {
	double rng = distance + distinc;
	if(rng < 0.1) rng = 0.1; 
	// now go to the remote point
	Point next; 
	stepTo(az, rng, next); 

	// and calculate the distance between that and "other"
	double d_err = other.distanceTo(next);
	if(d_err < err_dist) {
	  err_dist = d_err; 
	  new_bearing = az; 
	  new_distance = rng; 
	  got_better = true; 
	  ret = true; 
	}
	if(d_err > last_err) break; // we're getting worse. 
	last_err = d_err; 
      }
      // if we didn't improve on the last azimuth, bail out
      if(!got_better) break; 
    }

    return (err_dist < 0.01); // if we're within 10m, let's quit.
  }
    
  void Point::correctBearingDistanceTo(const Point & other, 
				       const double bearing, 
				       const double distance, 
				       double & new_bearing, 
				       double & new_distance) const {
    new_bearing = bearing;
    new_distance = distance;
    double b_span = 1.0;
    double d_span = 30.0; 
    for(int i = 0; i < 16; i++) {
      if(recCorrectBearingDistanceTo(other, new_bearing, new_distance, b_span, d_span, new_bearing, new_distance)) return;
      b_span = b_span * 0.25;
      d_span = d_span * 0.25; 
      if((b_span < 0.01) && (d_span < 0.01)) break; 
    }
  }
  
  double Point::inSpan(double ang) const {
    while((ang < 0.0) || (ang >= (2.0 * M_PI))) {
      if(ang < 0.0) {
	ang += 2.0 * M_PI; 
      }
      else {
	ang -= 2.0 * M_PI; 
      }
    }
    return ang;
  }

  __attribute__((optimize("-fno-fast-math")))  
  void Point::stepTo(double bearing, double distance, Point & next) const {

    /* Initialized data */
    const double eps = 5e-14;
    const double a = 6378206.4; /* (meters) */
    const double f = 1.0/298.25722210088; 

    /* System generated locals */
    double d1;

    bool debug = ((bearing < 180.00001d) && (bearing > 179.99999d)) || 
      (bearing < 0.00001d) || (bearing > 359.99999d);

    /* Local variables */
    double c, d, e, r, x, y, cf, sa, cu, sf, cy, cz, su, tu, 
      sy, c2a, s, faz, rbaz;

    double glat1, glon1, glat2, glon2; 

    // *** SOLUTION OF THE GEODETIC DIRECT PROBLEM AFTER T.VINCENTY 
    // *** MODIFIED RAINSFORD'S METHOD WITH HELMERT'S ELLIPTICAL TERMS 
    // *** EFFECTIVE IN ANY AZIMUTH AND AT ANY DISTANCE SHORT OF ANTIPODAL 
    // 
    // *** A IS THE SEMI-MAJOR AXIS OF THE REFERENCE ELLIPSOID 
    // *** F IS THE FLATTENING OF THE REFERENCE ELLIPSOID 
    // *** LATITUDES AND LONGITUDES IN RADIANS POSITIVE NORTH AND EAST 
    // *** AZIMUTHS IN RADIANS CLOCKWISE FROM NORTH 
    // *** GEODESIC DISTANCE S ASSUMED IN UNITS OF SEMI-MAJOR AXIS A 
    // 
    // *** PROGRAMMED FOR CDC-6600 BY LCDR L.PFEIFER NGS ROCKVILLE MD 20FEB75 
    //      
    // *** MODIFIED FOR SYSTEM 360 BY JOHN G GERGEN NGS ROCKVILLE MD 750608
    // 
    // *** HACKED TO HELL AND GONE BY F2C (July-17-96 version) and by 
    // *** Matt Reilly (KB1VC) to make it fit with the dem-gridlib routines.
    // *** Feb 24, 1997. 
    // *** pounded on again by Matt Reilly (kb1vc) June 2019 for the GeoProfII
    // *** code 

    // distance is in Km... */ 
    s = distance * 1000.0;
    faz = bearing * M_PI / 180.0;

    glat1 = this->lat * M_PI / 180.0;
    glon1 = this->lon * M_PI / 180.0; 
    
    r = 1.0 - f;
    
    tu = r * sin(glat1) / cos(glat1);
    
    sf = sin(faz);
    cf = cos(faz);
    rbaz = 0.0;
    if (cf != 0.0) {
      rbaz = atan2(tu, cf) * 2.0;
    }

    cu = 1.0 / sqrt(tu * tu + 1.0);
    su = tu * cu;
    sa = cu * sf;
    c2a = -sa * sa + 1.0;
    x = sqrt((1.0 / r / r - 1.0) * c2a + 1.0) + (float)
      1.;
    x = (x - 2.0) / x;
    c = 1.0 - x;
    c = (x * x / 4.0 + 1) / c;
    d = (x * .375 * x - 1.0) * x;
    tu = s / r / a / c;
    y = tu;
  L100:
    sy = sin(y);
    cy = cos(y);
    cz = cos(rbaz + y);
    e = cz * cz * 2.0 - 1.0;
    c = y;
    x = e * cy;
    y = e + e - 1.0;
    y = (((sy * sy * 4.0 - 3.0) * y * cz * d / 6.0 + x) * 
	 d / (float)4. - cz) * sy * d + tu;
    if (fabs(y - c) > eps) {
      goto L100;
    }
    rbaz = cu * cy * cf - su * sy;
    c = r * sqrt(sa * sa + rbaz * rbaz);
    d = su * cy + cu * sy * cf;
    glat2 = atan2(d, c);
    c = cu * cy - su * sy * cf;
    x = atan2(sy * sf, c);
    c = ((c2a * -3.0 + 4.0) * f + 4.0) * c2a * 
      f / 16.0;
    d = ((e * cy * c + cz) * sy * c + y) * sa;
    glon2 = glon1 + x - (1.0 - c) * d * f;

    next.lat = glat2 * 180.0 / M_PI;
    next.lon = glon2 * 180.0 / M_PI; 
    
    if(next.lon < 180.0) next.lon = 360.0 + next.lon;
    if(next.lon > 180.0) next.lon = next.lon - 360.0;
    return; 
    
  }

  void Point::pt2Grid(std::string & grid) const {
    std::string ret(6, ' ');

    // First convert lat and lon to degrees and minutes.
    // Remember the basis is lon 180 is Grenwich, lat 0 is the pole
    // so we need to correct;
    double llon = lon + 180;
    double llat = lat + 90;
    
    int ilat = ((int) floor(llat));
    int ilat_mins = ((int) floor(60.0 * (llat - floor(llat))));

    int ilon = ((int) floor(llon));
    int ilon_mins = ((int) floor(60.0 * (llon - floor(llon))));

    // set lon positions
    ret[0] = 'A' + ((int) (ilon / 20));
    ilon = ilon % 20;
    ret[2] = '0' + (ilon / 2);
    ilon = ilon % 2;
    ret[4] = 'a' + ((ilon * 60 + ilon_mins) / 5);
    
    ret[1] = 'A' + ((int) (ilat / 10));
    ilat = ilat % 10;
    ret[3] = '0' + ilat;
    ret[5] = 'a' + ((ilat_mins * 2) / 5);

    grid = ret; 

    return; 
  }


  void Point::dms2Pt(double lat_d, double lat_m, double lat_s, char ns,
		      double lon_d, double lon_m, double lon_s, char ew) {
    lat = lat_d + (lat_m / 60.0) + (lat_s / 3600.0);
    if((ns == 'S') || (ns == 's')) lat = -1 * lat;
    
    lon = lon_d + (lon_m / 60.0) + (lon_s / 3600.0);    
    if((ew == 'W') || (ew == 'w')) lon = -1 * lon; 
  }

  double Point::gridDiff(char v, char s, double mul) const {
    return mul * ((double) (v - s));
  }
  
  void Point::grid2Pt(const std::string & grid) {
    if(!checkGrid(grid)) {
      throw std::runtime_error((boost::format("Bad grid specifier: [%s] is not in the proper form.") % grid).str());
    }

    std::string lgrid = grid;

    // upcase all of it... Is anybody embarrassed by the awkwardness here?
    std::transform(lgrid.begin(), lgrid.end(), lgrid.begin(), ::toupper);


    lon = gridDiff(lgrid[0], 'A', 20.0) + 
      gridDiff(lgrid[2], '0', 2.0);

    if(lgrid.size() > 4) {
      lon += (gridDiff(lgrid[4], 'A', 5.0) + 2.5) / 60.0; 
    }
    
    lon = lon - 180.0; 

    
    lat = gridDiff(lgrid[1], 'A', 10.0) + 
      gridDiff(lgrid[3], '0', 1.0);

    if(lgrid.size() > 5) {
      lat += (gridDiff(lgrid[5], 'A', 2.5) + 1.25) / 60.0; 
    }

    lat = lat - 90.0; // correct for south pole being 0deg lat.
  }

  
  bool Point::checkGrid(const std::string & grid) const
  {
    //  verifies that the grid square is legitimate
    //  return true for a good grid. 

    return regex_match(grid, grid_regexp); 
  }


  double Point::atan2Pt(double y, double x) const {
    double retval;
    
    retval = atan2(y, x);
    if(retval < 0.0) retval = ((2.0 * M_PI) + retval);
    
    return retval;
  }
  
}
