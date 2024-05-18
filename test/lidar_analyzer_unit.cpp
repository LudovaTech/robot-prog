#include <gtest/gtest.h>

#include "../src/lidar_analyzer.h"



TEST(AnalyzeLidarData, filterDistance) {
  AnalyzeLidarData ald; 
  ASSERT_TRUE(ald.filterDistance(LidarPoint(500, 0, 0)));
  ASSERT_TRUE(ald.filterDistance(LidarPoint(1500, 0, 0)));
  ASSERT_FALSE(ald.filterDistance(LidarPoint(5000, 0, 0)));
}

TEST(AnalyzeLidarData, convCoordonneesCartesiennes) {
  AnalyzeLidarData ald;
  MutableVector2* ref_tab = ald._getConvPoints();
  ASSERT_TRUE(ald.convCoordonneesCartesiennes(LidarPoint(0, 0, 0), 0));
  ASSERT_TRUE(ald.convCoordonneesCartesiennes(LidarPoint(10, 5, 20), 1));
  ASSERT_TRUE(ald.convCoordonneesCartesiennes(LidarPoint(1000, 10, 0), 2));
  ASSERT_EQ(ref_tab[0], MutableVector2(0, 0)) << "ref_tab[0] is " << ref_tab[0].toString();
  ASSERT_FLOAT_EQ(ref_tab[1].x(), 9.9999390765779) << "ref_tab[1] is " << ref_tab[1].toString();
  ASSERT_FLOAT_EQ(ref_tab[1].y(), -0.034906514152237) << "ref_tab[1] is " << ref_tab[1].toString();
  ASSERT_EQ(ref_tab[2], MutableVector2(1000, 0)) << "ref_tab[2] is " << ref_tab[2].toString();
}