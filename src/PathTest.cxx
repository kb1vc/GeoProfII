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



int main(int argc, char ** argv)
{
  std::string from_grid = generateRandomGrid();
  std::string to_grid = generateOtherGrid(from_grid);

  from_grid = std::string("IM00aa");
  to_grid = std::string("JN99xx");

  std::cerr << boost::format("From %s To %s\n") % from_grid % to_grid;
  GeoProf::Point from_pt(from_grid);
  GeoProf::Point to_pt(to_grid);

  GeoProf::Path path(from_pt, to_pt, 1.0);

  for(auto p: path) {
    std::cerr << boost::format("%f %f\n") % p.getLatitude() % p.getLongitude();
  }
}

