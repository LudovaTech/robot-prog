#include <gtest/gtest.h>

#include "../src/lidar_analyzer.h"



TEST(AnalyzeLidarData, filterDistance) {
  AnalyzeLidarData ald; 
  ASSERT_TRUE(ald.filterDistance(LidarPoint(10, 0, 0)));
  ASSERT_TRUE(ald.filterDistance(LidarPoint(50, 0, 0)));
  ASSERT_FALSE(ald.filterDistance(LidarPoint(5000, 0, 0)));
}

TEST(AnalyzeLidarData, convCoordonneesCartesiennes) {
  AnalyzeLidarData ald;
  MutableVector2* ref_tab = ald._getConvPoints();
  ASSERT_TRUE(ald.convCoordonneesCartesiennes(LidarPoint(0, 0, 0), 0));
  ASSERT_TRUE(ald.convCoordonneesCartesiennes(LidarPoint(10, 5, 20), 0));
  ASSERT_TRUE(ald.convCoordonneesCartesiennes(LidarPoint(1000, 10, 0), 0));
  ASSERT_EQ(ref_tab[0], MutableVector2(0, 0));
  ASSERT_EQ(ref_tab[1], MutableVector2(0, 0));
  ASSERT_EQ(ref_tab[2], MutableVector2(0, 0));
}