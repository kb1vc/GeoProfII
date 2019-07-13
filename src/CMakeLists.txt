set(GeoProf_SRCS
  Point.cxx
  Path.cxx
  DEM.cxx
  DEMTile.cxx
  ElevationTile.cxx
  )

set(PointTest_SRCS
  PointTest.cxx)

set(PathTest_SRCS
  PathTest.cxx)


find_package(ZLIB REQUIRED)
find_package(Boost COMPONENTS  iostreams)
include_directories( ${ZLIB_INCLUDE_DIRS} )

add_library(GeoProf ${GeoProf_SRCS})
add_library(TestUtils TestUtils.cxx)

add_executable(PointTest ${PointTest_SRCS})

target_link_libraries(PointTest LINK_PUBLIC
  GeoProf TestUtils
  ${Boost_LIBRARIES})

add_executable(PathTest ${PathTest_SRCS})

target_link_libraries(PathTest LINK_PUBLIC
  GeoProf TestUtils  
  ${Boost_LIBRARIES})

set(CMAKE_EXE_LINKER_FLAGS 
  "-static-libgcc -static-libstdc++")

add_executable(DEMTest DEMTest.cxx)


target_link_libraries(DEMTest LINK_PUBLIC
  GeoProf TestUtils
  ${Boost_LIBRARIES}
  ${ZLIB_LIBRARIES})



  
