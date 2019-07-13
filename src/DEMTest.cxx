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
#include <list>


int main(int argc, char ** argv)
{
  //  std::list<std::string> fname_list;
  // fname_list.push_back(std::string(argv[1]));
  // GeoProf::ElevationDB<GeoProf::DEMTile> dbase(fname_list); 
  std::ifstream file_list_stream(argv[1]);
  GeoProf::ElevationDB<GeoProf::DEMTile> dbase(file_list_stream);
  file_list_stream.close();

  std::list<GeoProf::Point> pt_list; 
  pt_list.push_back(GeoProf::Point(45.3, -98.2));
  pt_list.push_back(GeoProf::Point(45.4, -98.2));
  pt_list.push_back(GeoProf::Point(45.5, -98.2));  
  pt_list.push_back(GeoProf::Point(42.48893, -71.8868));
  pt_list.push_back(GeoProf::Point(42.45, -71.85));
  pt_list.push_back(GeoProf::Point(42.45, -71.90));  

  
  for(auto & pt : pt_list) {
    double elev; 
    if(dbase.getElevation(pt, elev)) {
      std::cerr << boost::format("Point %s : Elevation %g\n")
	% pt.toString() % elev;
    }
    else {
      std::cout << boost::format("Could not find point %s\n")
	% pt.toString();
    }
  }
	     
}

