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
#include "ElevationDB.hxx"

void GeoProf::ElevationDB_Base::makeDB(std::istream & lstr, const std::string & sav_name, const std::string & map_typename) {

  boost::iostreams::filtering_ostream out;
  out.push(boost::iostreams::gzip_compressor(boost::iostreams::gzip_params(boost::iostreams::gzip::best_compression)));
  out.push(boost::iostreams::file_descriptor_sink(sav_name)); 

  std::vector<std::string> fnames; 
  std::string tile_file_name; 
  while(lstr >> tile_file_name) {
    fnames.push_back(tile_file_name); 
  }

  unsigned int num_tiles = fnames.size();
  out.write((char*)&num_tiles, sizeof(num_tiles));

  char typnamebuf[256]; 
  out.write((char*)&typnamebuf, 256);

  int count = 0; 
  for(auto & tfn: fnames) {
    count++; 
    ElevationTile * tile_p = makeTile(tfn);
	
    tile_p->save(out);
	
    deleteTile(tile_p);
  }

  boost::iostreams::close(out);
  
}

void GeoProf::ElevationDB_Base::scanPath(GeoProf::Path & path, std::vector<short> & elevations) {
  elevations.clear();
  for(auto pt: path) {
    short elev;
    if(getElevation(pt, elev)) {
      elevations.push_back(elev);	  
    }
    else {
      elevations.push_back(0);
    }
  }
}

void GeoProf::ElevationDB_Base::restore(const std::string & fname) {

  std::ifstream compressed_istream(fname, std::ios_base::in | std::ios_base::binary);
  boost::iostreams::filtering_streambuf<boost::iostreams::input> inbuf;          
  inbuf.push(boost::iostreams::gzip_decompressor());
  inbuf.push(compressed_istream);

  std::istream is(&inbuf);

  restore(is); 

  compressed_istream.close();
}


void GeoProf::ElevationDB_Base::restore(std::istream & is) {
  unsigned int mapsize; 
  is.read((char*)&mapsize, sizeof(mapsize));
  char tile_type_name_buf[256];
  is.read(tile_type_name_buf, 256);

      
  for(int i = 0; i < mapsize; i++) {
    ElevationTile * tile_p = makeTile();
    tile_p->restore(is);
    registerTile(tile_p);
  }
}

void GeoProf::ElevationDB_Base::registerTile(ElevationTile * tile_p) {
  // do any "cleanup" or preparation that the tile object might need. 
  tile_p->prepare();

  BoundingBox bb = tile_p->getBoundingBox(); 

  tile_map[bb] = tile_p;
}  

bool GeoProf::ElevationDB_Base::getElevation(const Point & point, short & elev) {
  ElevationTile * tile = findTile(point); 
  if(tile == NULL) {
    return false; 
  }
      
  return tile->getElevation(point, elev);

  return true; 
}


GeoProf::ElevationTile * GeoProf::ElevationDB_Base::findTile(const Point & point) {
  BoundingBox bb(point);

  for(auto & me : tile_map) {
    if(me.first.isIn(point)) return me.second;
  }
  return NULL;
}


bool GeoProf::ElevationDB_Base::findHighPoint(const GeoProf::Point & point, 
					      GeoProf::Point & high_point, 
					      double radius, double step) {
  short max_el;
  high_point = point; 
  if(!getElevation(point, max_el)) return false; 
  short was_el = max_el;

  // now look everwhere in a 500 meter radius of this point.
  std::vector<short> elevs;
  for(double ang = 0.0; ang < 360.0; ang += 5.0) {
    Point next; 
    point.stepTo(ang, radius, next); 
    GeoProf::Path path(point, next, step);
    scanPath(path, elevs);
    int i = 0; 
    for(auto el: elevs) {
      if(el > max_el) {
	high_point = next; 
	max_el = el; 
      }
      i++; 
    }
  }

  return true; 
}


void GeoProf::ElevationDB_Base::save(const std::string & fname) {
  boost::iostreams::filtering_ostream out;
  out.push(boost::iostreams::gzip_compressor(boost::iostreams::gzip_params(boost::iostreams::gzip::best_compression)));
  out.push(boost::iostreams::file_descriptor_sink(fname)); 

  save(out);

  boost::iostreams::close(out);
}

void GeoProf::ElevationDB_Base::save(std::ostream & os) {
  // save format is
  // [unsigned int] sizeof_map
  // [char [256] ] tile_type_name
  // one TileClass instance after another.

  unsigned int sizeof_map = tile_map.size();
  char tile_type_name_buf[256];

  os.write((char*)&sizeof_map, sizeof(unsigned int));
  os.write((char*)&tile_type_name_buf, 256); 

  for(auto & tile : tile_map) {
    tile.second->save(os);
  }
}
