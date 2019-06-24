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
#include <boost/format.hpp>
#include <iostream>
#include <math.h>

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
	      tll.toGrid(tgrid);
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
  
  std::cout << boost::format("LatLon from grid [%f, %f] error = [%f, %f]\n")
    % n_gr2ll.getLatitude() % n_gr2ll.getLongitude() % lat_err % lon_err; 

  gridLoop();
}

