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
#include <regex>

std::regex GeoProf::DEM::float_cleanup_re = std::regex("([0-9]E[+-][0-9][0-9])");

void GeoProf::DEM::readHeader() {
  // read the first 1024 bytes
  std::string ibuf(1024, ' ');
  inf.get(&ibuf[0], 1024); // this contains the whole header.

  // FORTRAN shows double precision floats as n.nnnnnnnDsnn  change the D to an E
  std::regex re("([0-9])D([+-][0-9])");
  std::string buf = std::regex_replace(ibuf, re, "$1E$2");
  // now cut it up.
  extract(buf, 1, 40, file_name);
  extract(buf, 41, 80, description);
  std::vector<double> quadvec; 
  extractVector(buf, 547, 738, quadvec, 8);
  // convert to points
  for(int i = 0; i < 4; i++) {
    corners[i] = Point(quadvec[i * 2 + 1] / 3600.0, quadvec[i * 2] / 3600.0, 0.0);
  }
  std::vector<double> resolutionvec;
  extractVector(buf, 817, 852, resolutionvec, 3);
  std::vector<unsigned int> dims; 
  extractVector(buf, 853, 864, dims, 2);

  std::cerr << boost::format("file_name [%s]\ndescription[%s]\ndims (%d %d)\n")
    % file_name % description % dims[0] % dims[1];

  for(int i = 0; i < 4; i++) {
    std::cerr << boost::format("Corner[%d] = [%f %f]\n") % i % corners[i].getLatitude() % corners[i].getLongitude();
  }

  for(int i = 0; i < 3; i++) {
    std::cerr << resolutionvec[i] << std::endl; 
  }
}

