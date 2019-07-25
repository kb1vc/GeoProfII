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
#include "TestUtils.hxx"
#include <boost/format.hpp>
#include <iostream>
#include <cmath>

// return true on failed test.
bool testBearingDist(unsigned int c) {
  // generate two grids.
  std::string from_grid = generateRandomGrid();
  std::string to_grid = generateOtherGrid(from_grid);

  // If we ever need a juicy test case -- 
  // from_grid = "PQ43jl";
  // to_grid = "PH43jl";
  
  // create the two points
  GeoProf::Point from_pt(from_grid);
  GeoProf::Point to_pt(to_grid);
  
  // make the forward and reverse bearing
  double bearing, rev_bearing, distance; 

  // give the bearing along the line from_pt to to_pt. 
  from_pt.bearingDistanceTo(to_pt, bearing, rev_bearing, distance);
  
  // if the distance is less than 2km -- just return success
  if(distance < 2.0) return false;
  
  // correct the bearings
  from_pt.correctBearingDistanceTo(to_pt, bearing, distance, bearing, distance);

  double nd; 
  to_pt.correctBearingDistanceTo(from_pt, rev_bearing, distance, rev_bearing, nd);

  // now find the point at the other end by traveling that distance along the bearing.
  GeoProf::Point nto_pt;
  from_pt.stepTo(bearing, distance, nto_pt);

  // do the reverse bearing
  GeoProf::Point nfrom_pt;
  to_pt.stepTo(rev_bearing, distance, nfrom_pt);

  // how far apart are the generated points from the original points?
  double from_distance, to_distance, dmy, dmy2;
  from_pt.bearingDistanceTo(nfrom_pt, dmy, dmy2, from_distance);
  to_pt.bearingDistanceTo(nto_pt, dmy, dmy2, to_distance);  

  
  bool ret = false; 

  double dist_lim = 0.01 * distance;  // two percent
  
  if((fabs(from_distance) > dist_lim) || (fabs(to_distance) > dist_lim)) {
    std::string nto_gr;
    std::string nfrom_gr;     
    nto_pt.pt2Grid(nto_gr);
    nfrom_pt.pt2Grid(nfrom_gr);
    std::cerr << boost::format("%8d: f: %s  nfrom: %s (dist %f) t: %s nto: %s (dist %f) path length %f bearing %f\n")
      % c % from_grid % nfrom_gr % from_distance % to_grid % nto_gr % to_distance % distance % bearing;

    std::cerr << boost::format("\tf: [%f %f] nfrom: [%f %f]\n")
      % from_pt.getLatitude() % from_pt.getLongitude()
      % nfrom_pt.getLatitude() % nfrom_pt.getLongitude();
    std::cerr << boost::format("\tt: [%f %f] nto: [%f %f]\n")
      % to_pt.getLatitude() % to_pt.getLongitude()
      % nto_pt.getLatitude() % nto_pt.getLongitude();
    ret = true;    
  }

  if(ret) {
    std::cerr << boost::format("Failed path test From %s to %s\n") % from_grid % to_grid;
  }
  
  
  return ret; 
}

void gridLoop()
{
  int error_count = 0; 
  std::string grid(6, ' ');
  for(char la0 = 'A'; la0 <= 'R'; la0++) {
    grid[0] = la0; 
    for(char lo0 = 'A'; lo0 <= 'R'; lo0++) {
      grid[1] = lo0;       
      for(char la1 = '0'; la1 <= '9'; la1++) {
	grid[2] = la1;       	
	for(char lo1 = '0'; lo1 <= '9'; lo1++) {
	  grid[3] = lo1;       		  
	  for(char la2 = 'a'; la2 <= 'x'; la2++) {
	    grid[4] = la2;       		  	    
	    for(char lo2 = 'a'; lo2 <= 'x'; lo2++) {
	      grid[5] = lo2;       		  	      
	      GeoProf::Point tll(grid);
	      std::string tgrid;
	      tll.pt2Grid(tgrid);
	      if(tgrid != grid) {
		std::cerr << boost::format("grid %s => [%f %f] => grid %s.\n")
		  % grid % tll.getLatitude() % tll.getLongitude() % tgrid;
		error_count++; 
	      }
	    }
	  }
	}
      }
    }
  }
  std::cout << boost::format("%d errors in grid translation check.\n") % error_count; 
}


int main(int argc, char ** argv)
{
  GeoProf::Point n_ll(41.71463, -72.72713);
  GeoProf::Point n_gr2ll("FN31pr");

  double lat_err = n_gr2ll.getLatitude() - n_ll.getLatitude();
  double lon_err = n_gr2ll.getLongitude() - n_ll.getLongitude();
  

  unsigned long fail_count = 0; 
  unsigned long it_count = 1024 * 32;
  for(unsigned long i = 0; i < it_count; i++) {    
    if (testBearingDist(i)) {
      fail_count++;
    }
  }

  std::cerr << boost::format("Tested %d pairs, got %d failures\n") % it_count % fail_count;
  //   gridLoop();

}

