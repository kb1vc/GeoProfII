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

#include "DEM.hxx"
#include "DEMTile.hxx"
#include "ElevationDB.hxx"
#include <boost/format.hpp>
#include <iostream>
#include <cmath>
#include <random>



int main(int argc, char ** argv)
{
  std::string fname(argv[1]);
  GeoProf::ElevationDB<GeoProf::DEMTile> dbase; 
  GeoProf::DEM dem(fname, dbase);
 
  std::cerr << dem.getName() << std::endl; 
  
  GeoProf::Point test_pt(45.3, -98.2);
  
  double elev; 
  if(dbase.getElevation(test_pt, elev)) {
    std::cout << boost::format("Point %s : Elevation %g\n")
      % test_pt.toString() % elev;
  }
  else {
    std::cout << boost::format("Could not find point %s\n")
      % test_pt.toString();
  }
	     
}

