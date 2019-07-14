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
#include "Path.hxx"
#include "TestUtils.hxx"
#include <boost/format.hpp>
#include <iostream>
#include <cmath>
#include <random>
#include "DEMTile.hxx"
#include "ElevationDB.hxx"


int main(int argc, char ** argv)
{
  std::string from_grid("FN42bl");
  std::string to_grid("FN44ig");

  std::cerr << boost::format("From %s To %s\n") % from_grid % to_grid;
  GeoProf::Point from_pt(from_grid);
  GeoProf::Point to_pt(to_grid);

  
  // now create a new dbase from the compressed file.
  GeoProf::ElevationDB<GeoProf::DEMTile> cdb;
  std::cerr << "about to restore.\n";
  cdb.restore(std::string(argv[1]));
  std::cerr << "restored\n";

  // find the high point
  GeoProf::Point new_frm, new_to;
  cdb.findHighPoint(from_pt, new_frm);
  cdb.findHighPoint(to_pt, new_to);  
  
  GeoProf::Path path(new_frm, new_to, 0.020);
  
  std::vector<short> elevations; 
  
  cdb.scanPath(path, elevations); 

  int i = 0;
  for(auto p: path) {
    std::cout << boost::format("%f %f %d\n") 
      % p.getLatitude() % p.getLongitude() % elevations[i];
    i++;
  }
}

