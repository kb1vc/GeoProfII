set(PointTest_SRCS
    PointTest.cxx
    Point.cxx)


add_executable(PointTest ${PointTest_SRCS})

target_link_libraries(PointTest
    ${Boost_LIBRARIES})

