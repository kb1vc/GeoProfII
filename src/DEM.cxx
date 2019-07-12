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

std::regex GeoProf::DEM::float_cleanup_re = std::regex("([0-9]E[+-][0-9][0-9])");
std::regex GeoProf::DEM::float_convert_re = std::regex("([0-9])D([+-][0-9])");

GeoProf::DEM::DEM(const std::string & fname, ElevationDB<DEMTile> & database) {
  openInstream(fname);

  // read the header.   See "Standards for Digital Elevation Models -- Part2 : Specifications -- Appendix 2-A
  readHeader();

  // create the DEMTile for this file
  dem_tile = new DEMTile(corners[0], corners[2], num_profiles);

  // read the profiles
  readProfiles();

  // now register the tile with the database
  database.registerTile(dem_tile);

  closeInstream();
}

void GeoProf::DEM::openInstream(const std::string & fname) {
  // from an example at https://techoverflow.net/2013/11/03/c-iterating-lines-in-a-gz-file-using-boostiostreams/
  
  compressed_istream.open(fname, std::ios_base::in | std::ios_base::binary);
  inbuf.push(boost::iostreams::gzip_decompressor());
  inbuf.push(compressed_istream); 

  // now make it an input stream
  instr_p =  new std::istream(&inbuf);
  
  std::string status[4]; 
  status[0] = instr_p->good() ? "Good" : "Not Good";
  status[1] = instr_p->eof() ? "EOF" : "Not EOF";
  status[2] = instr_p->fail() ? "FAIL" : "OK";
  status[3] = instr_p->bad() ? "Bad" : "Not Bad";
  
  for(int i = 0; i < 3; i++) {
    std::cerr << status[i] << ", "; 
  }
  std::cerr << std::endl;
}

void GeoProf::DEM::closeInstream() {
  compressed_istream.close();
}


void GeoProf::DEM::readProfiles() {
  for(unsigned int i = 0; i < num_profiles; i++) {
    readSingleProfile(i);
  }
}

void GeoProf::DEM::readSingleProfile(unsigned int xidx)
{
  // The profile contains FORTRAN format double precision numbers
  // with "D" in the exponent marker.  That means that we need to parse
  // the header from a stream.
  int row, col; 
  (*instr_p) >> row >> col;
  int m, n;
  (*instr_p) >> m >> n;

  std::vector<double> & elev = dem_tile->getProfile(col);
  
  double lon, lat;
  lon = readDouble((*instr_p));
  lat = readDouble((*instr_p)); 

  elev.resize(m);
  
  std::cerr << boost::format("row = %d col = %d m = %d n = %d\n")
    % row % col % m % n;

  double dat_el, min_el, max_el; 
  dat_el = readDouble((*instr_p));
  max_el = readDouble((*instr_p));
  min_el = readDouble((*instr_p));  

  for(int i = 0; i < m; i++) {
    (*instr_p) >> elev[i];
    if(i < 10) std::cerr << elev[i] << std::endl; 
  }

  if((m + n) > 10000) exit(-1);
}

void GeoProf::DEM::readHeader() {
  // read the first 1024 bytes
  std::string buf(1024, ' ');
  instr_p->get(&buf[0], 1024); // this contains the whole header.

  // FORTRAN shows double precision floats as n.nnnnnnnDsnn  change the D to an E
  convertDFloat2EFloat(buf);

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
  num_profiles = dims[1];

  // we now know how big the 
}

