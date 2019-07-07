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
#include "TestUtils.hxx"

std::default_random_engine gen;
std::uniform_int_distribution<char> char01_dist('A','R');
std::uniform_int_distribution<char> char23_dist('0', '9');
std::uniform_int_distribution<char> char45_dist('a', 'x');
std::uniform_int_distribution<int> mask_dist(0, 63);

std::string generateRandomGrid()
{
  std::string ret(6, ' ');
  ret[0] = char01_dist(gen);
  ret[1] = char01_dist(gen);
  ret[2] = char23_dist(gen);
  ret[3] = char23_dist(gen);
  ret[4] = char45_dist(gen);
  ret[5] = char45_dist(gen);  

  return ret; 
}

std::string generateOtherGrid(std::string in)
{
  int msk = mask_dist(gen);
  std::string ret(6, ' '); 
  ret[0] = (msk & 1) ? in[0] : char01_dist(gen);
  ret[1] = (msk & 2) ? in[1] : char01_dist(gen);
  ret[2] = (msk & 4) ? in[2] : char23_dist(gen);
  ret[3] = (msk & 8) ? in[3] : char23_dist(gen);
  ret[4] = (msk & 16) ? in[4] : char45_dist(gen);
  ret[5] = (msk & 32) ? in[5] : char45_dist(gen);   
  
  if(ret == in) return generateOtherGrid(in);
  else return ret; 
}
