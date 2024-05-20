#include <string>

#include "lidar.h"
#include "lidar_analyzer.h"
#include "logger.h"
#include "movements.h"
#include "states.h"
#include "strategy.h"
#include "utilities.h"

const FieldProperties fieldProperties = FieldProperties(
    243,               // fieldLength
    182,               // fieldWidth
    12,                // spaceBeforeLineSide
    60,                // goalWidth
    Vector2(0, -115),  // myGoalPos
    Vector2(0, 115),   // enemyGoalPos
    9,                 // robotRadius
    2                  // ballRadius
);

const Motors motors = Motors(

    // Arduino UNO
    // MotorMov(11, 12, 0, Degree(-40)),
    // MotorMov(5, 4, 0, Degree(40)),
    // MotorMov(6, 7, 0, Degree(-140)),
    // MotorMov(9, 8, 0, Degree(140))

    // Teensy
    MotorMov(15, 14, 0, Degree(-40)),
    MotorMov(36, 33, 0, Degree(40)),
    MotorMov(22, 19, 0, Degree(-140)),
    MotorMov(11, 12, 0, Degree(140)));

RobotState currentState = RobotState(
    Vector2(0, 0),
    Vector2(0, 0),
    Vector2(0, 0),
    Vector2(0, 0),
    Vector2(0, 0));

///////////

CircularLidarPointsBuffer parseAndPrintPairs(String input) {
  CircularLidarPointsBuffer buffer(AnalyzeLidarData::nbrLidarPoints);
  unsigned int index = 0;
  SerialDebug.println("index : ");
  while (index < input.length()) {
    SerialDebug.print("b");
    // Trouver les indices des parenthèses
    int startIdx = input.indexOf('(', index);
    int endIdx = input.indexOf(')', startIdx);

    // Si on ne trouve plus de parenthèses, arrêter la boucle
    if (startIdx == -1 || endIdx == -1) {
      break;
    }
    SerialDebug.print("c");
    // Extraire la sous-chaîne correspondant à une paire (x, y)
    String pair = input.substring(startIdx + 1, endIdx);
    SerialDebug.print("c1");
    // Trouver la virgule qui sépare x et y
    int commaIdx = pair.indexOf(',');
    SerialDebug.print("c2");
    // Extraire x et y en tant que sous-chaînes
    String xStr = pair.substring(0, commaIdx);
    SerialDebug.print("c3");
    String yStr = pair.substring(commaIdx + 1);
    SerialDebug.print("d");
    // Convertir les sous-chaînes en entiers
    int x = xStr.toInt();
    int y = yStr.toInt();
    //SerialDebug.println(index);
    buffer.addValue(
        LidarPoint(
            x,
            0,
            y));
    SerialDebug.print("e");
    // Mettre à jour l'index pour le prochain tour de boucle
    index = endIdx + 1;
  }
  SerialDebug.println("DONE");
  return buffer;
}

bool checkCoordinates(AnalyzeLidarData ald, FieldProperties fP, String text, double x, double y, String input) {
  SerialDebug.println("checkCoordinates started");
  CircularLidarPointsBuffer buffer = parseAndPrintPairs(input);
  SerialDebug.println("in buffer");
  SerialDebug.println(buffer.toString());
  SerialDebug.print("result : ");
  SerialDebug.println(String(bool(ald.convFromBuffer(buffer))));
  /*
  MutableLidarPoint buf[2] = {
    MutableLidarPoint(
      1,2,3
    ),
    MutableLidarPoint(
      4,5,6
    )
  }
  distanceMax = ald.getDistanceMax();
  for (unsigned int i = 0; i < 2; i++) {
    LidarPoint lidarPoint = buf[i];
    if (ald.filterDistance(lidarPoint)) {
      distanceMax = max(distanceMax, lidarPoint.distance());
      if (!ald.convCoordonneesCartesiennes(lidarPoint, i)) {
        return false;
      }
    }
  }*/

  SerialDebug.println("converted");
  ald.findWalls(fP);
  SerialDebug.println("find walls done");
  LidarInfos infos = ald.getLidarInfos();
  SerialDebug.println(infos.toString());

  double tolerance = 8;  // cm
  Vector2 robotCoordinates = infos.coordinates();

  if (robotCoordinates.x() / 10.0 > x - tolerance && robotCoordinates.x() / 10.0 < x + tolerance && robotCoordinates.y() / 10.0 > y - tolerance && robotCoordinates.y() / 10.0 < y + tolerance) {
    double distanceTest = Vector2(robotCoordinates.x() / 10.0, robotCoordinates.y() / 10.0).distance(Vector2(x, y));
    double successPercentage = ((tolerance - distanceTest) / tolerance) * 100.0;
    SerialDebug.println("SUCCESS - " + text + " x=" + String(robotCoordinates.x() / 10.0) + ", y=" + String(robotCoordinates.y() / 10.0) + ", " + String(successPercentage) + "%");
    return true;
  } else {
    SerialDebug.println("FAILURE - " + text + " -> Expecting: x=" + String(x) + ", y=" + String(y) + " ; got: x=" + String(robotCoordinates.x() / 10.0) + ", y=" + String(robotCoordinates.y() / 10.0));
    return false;
  }
}

FieldProperties classicFP() {
  return FieldProperties(
      243,               // fieldLength
      182,               // fieldWidth
      12,                // spaceBeforeLineSide
      60,                // goalWidth
      Vector2(0, -115),  // myGoalPos
      Vector2(0, 115),   // enemyGoalPos
      9,                 // robotRadius
      2                  // ballRadius
  );
}

///////////

void setup() {
  SerialDebug.begin(115200);
  SerialCam.begin(115200);
  SerialLidar.begin(230400);

  SerialCam.setTimeout(10);
  SerialLidar.setTimeout(10);

  setupLog(ErrorLevel, true);
}

void loop() {
  AnalyzeLidarData ald;
  //String log = "(5564,1480);(5635,1489);(5706,1497);(5777,1506);(5848,1517);(5919,1526);(5990,1533);(6061,1540);(6132,1551);(6203,1565);(6274,1580);(6345,1596);(31547,464);(31620,466);(31693,467);(31766,469);(31839,471);(31912,473);(31985,475);(32058,478);(32131,480);(32204,482);(32277,485);(32350,487);(32424,490);(32494,492);(32564,496);(32634,499);(32704,503);(32774,506);(32844,510);(32914,514);(32984,522);(33054,527);(33124,533);(33194,539);(33275,545);(33346,551);(33417,556);(33488,561);(33559,566);(33630,571);(33701,573);(33772,576);(33843,580);(33914,584);(33985,591);(34056,596);(34131,601);(34207,608);(34283,617);(34359,625);(34435,633);(34511,640);(34587,647);(34663,655);(34739,663);(34815,671);(34891,678);(34967,686);(35047,695);(35118,704);(35189,712);(35260,719);(35331,725);(776,1090);(847,1110);(918,1140);(989,1169);(1060,1221);(1131,1255);(1202,1289);(1273,1325);(1344,1326);(1415,1328);(1486,1326);(1565,1322);(1641,1319);(1717,1316);(1793,1313);(1869,1311);(1945,1309);(2021,1307);(2097,1305);(2173,1304);(2249,1302);(2325,1301);(2401,1300);(2477,1300);(2544,1299);(2611,1299);(2678,1299);(2745,1299);(2812,1300);(2879,1300);(2946,1301);(3013,1302);(3080,1303);(3147,1304);(3214,1306);(3279,1308);(3350,1310);(3421,1310);(3492,1308);(3563,1306);(3634,1308);(3705,1312);(3776,1316);(3847,1320);(3918,1324);(3989,1328);(4060,1332);(4138,1339);(4209,1344);(4280,1350);(4351,1355);(4422,1361);(4493,1367);(4564,1373);(4635,1380);(4706,1387);(4777,1394);(4848,1402);(4919,1409);(4990,1416);(5061,1424);(5132,1431);(5203,1439);(5274,1446);(5345,1454);(5416,1461);(5487,1468);(5558,1476);(5629,1484);(5700,1492);(5771,1503);(5850,1514);(5921,1525);(5992,1533);(6063,1539);(6134,1546);(6205,1558);(6276,1571);(6347,1590);(6418,1606);(6489,1620);(6560,1638);(6631,1655);(6705,1672);(6776,1689);(6847,1707);(6918,1728);(6989,1747);(7060,1764);(7131,1779);(7202,1797);(7273,1822);(7344,1848);(7415,1874);(7486,1902);(7568,1930);(7639,1957);(7710,1989);(7781,2021);(7852,2050);(7923,2085);(7994,2119);(8065,2161);(8136,2201);(8207,2242);(8278,2278);(8349,2317);(8424,2364);(8499,2415);(8574,2409);(8649,239);(8724,246);(8799,2334);(8874,2327);(8949,2318);(9024,2305);(9099,2291);(9174,2278);(9249,2267);(9333,2253);(9404,2240);(9475,2228);(9546,2217);(9617,2207);(9688,2195);(9759,2184);(9830,2172);(9901,2161);(9972,2152);(10043,2142);(10114,2134);(10192,2127);(10263,2120);(10334,2113);(10405,2106);(10476,2099);(10547,2093);(10618,2088);(10689,2082);(10760,2076);(10831,2070);(10902,2064);(10973,732);(11044,731);(11116,729);(11188,729);(11260,729);(11332,731);(11404,733);(11476,735);(11548,736);(11620,737);(11692,737);(11764,739);(11836,754);(11907,774);(11979,792);(12051,2030);(12123,2036);(12195,2038);(12267,2040);(12339,2042);(12411,2045);(12483,2048);(12555,2051);(12627,2054);(12699,2057);(12777,2059);(12848,2063);(12919,2067);(12990,2074);(13061,2075);(13132,2076);(13203,1984);(13274,1858);(13345,1791);(13416,1694);(13487,1643);(13558,1590);(13633,1530);(13709,1485);(13785,1437);(13861,1406);(13937,1363);(14013,1328);(14089,1302);(14165,1285);(14241,1239);(14317,1192);(14393,1158);(14469,1121);(14542,1089);(14613,1067);(14684,1043);(14755,1031);(14826,1021);(14897,998);(14968,966);(15039,949);(15110,930);(15181,910);(15252,891);(15323,874);(15404,857);(15475,842);(15546,834);(15617,827);(15688,819);(15759,807);(15830,793);(15901,780);(15972,768);(16043,756);(16114,745);(16185,735);(16260,726);(16332,717);(16404,709);(16476,702);(16548,694);(16620,686);(16692,677);(16764,670);(16836,662);(16908,655);(16980,647);(17052,640);(17133,633);(17204,623);(17275,610);(18830,521);(18902,519);(18974,518);(19046,520);(19118,521);(19190,521);(19262,519);(19334,518);(19406,516);(19478,515);(19550,515);(19622,514);(19694,513);(19764,512);(19834,511);(19904,510);(19974,509);(20044,509);(20114,508);(20184,507);(20254,507);(20324,506);(20394,506);(20464,506);(20549,506);(20619,506);(20689,506);(20759,506);(20829,506);(20899,506);(20969,506);(21039,506);(21109,507);(21179,507);(21249,508);(21319,508);(21401,509);(21472,510);(21543,511);(21614,512);(21685,514);(21756,515);(21827,517);(21898,518);(21969,519);(22040,521);(22111,523);(22182,525);(22260,527);(22326,529);(22392,531);(22458,533);(22524,535);(22590,537);(22656,539);(22722,541);(22788,543);(22854,545);(22920,547);(22986,550);(23065,553);(23136,556);(23207,559);(23278,563);(23349,567);(23420,571);(23491,575);(23562,579);(23633,583);(23704,587);(23775,591);(23846,598);(23925,603);(23994,609);(24063,614);(24132,620);(24201,626);(24270,633);(24339,639);(24408,646);(24477,652);(24546,659);(24615,666);(24684,673);(24762,680);(24834,688);(24906,695);(24978,700);(25050,701);(25122,701);(25194,696);(25266,688);(25338,678);(25410,668);(25482,658);(25554,649);(25626,641);(25696,633);(25766,625);(25836,618);(25906,610);(25976,603);(26046,596);(26116,589);(26186,583);(26256,577);(26326,572);(26396,567);(26477,562);(26548,556);(26619,551);(26690,549);(26761,545);(26832,541);(26903,537);(26974,533);(27045,529);(27116,525);(27187,521);(27258,517);(27333,514);(27404,510);(27475,506);(27546,503);(27617,500);(27688,497);(27759,494);(27830,492);(27901,489);(27972,487);(28043,485);(28114,483);(28192,480);(28264,478);(28336,476);(28408,474);(28480,472);(28552,470);(28624,468);(28696,466);(28768,464);(28840,462);(28912,461);(28984,460);(29062,459);(29133,457);(29204,456);(29275,454);(29346,453);(29417,452);(29488,451);(29559,450);(29630,449);(29701,448);(29772,448);(29843,448);(29925,447);(29994,447);(30063,446);(30132,446);(30201,446);(30270,446);(30339,446);(30408,446);(30477,446);(30546,446);(30615,447);(30684,447);(30766,448);(30837,449);(30908,450);(30979,451);(31050,452);(31121,453);(31192,454);(31263,456);(31334,458);(31405,460);(31476,462);(31547,464);(31622,465);(31699,467);(31776,469);(31853,471);(31930,473);(32007,475);(32084,477);(32161,480);(32238,483);(32315,485);(32392,488);(32469,490);(32549,493);(32620,496);(32691,499);(32762,503);(32833,507);(32904,511);(32975,515);(33046,522);(33117,527);(33188,534);(33259,541);(33330,547);(33409,553);(33480,559);(33551,564);(33622,566);(33693,569);(33764,573);(33835,577);(33906,580);(33977,584);(34048,591);(34119,597);(34190,603);(34268,610);(34338,619);(34408,627);(34478,635);(34548,642);(34618,649);(34688,656);(34758,663);(34828,671);(34898,679);(34968,687);(35038,694);(35119,703);(35190,713);(35261,720);(35332,721);(772,1090);(843,1112);(914,1139);(985,1167);(1056,1220);(1127,1253);(1198,1281);(1269,1323);(1340,1325);(1411,1327);(1482,1326);(1553,1322);(1638,1319);(1714,1315);(1790,1313);(1866,1310);(1942,1307);(2018,1305);(2094,1304);(2170,1303);(2246,1302);(2322,1301);(2398,1300);(2474,1300);(2548,1300);(2614,1299);(2680,1299);(2746,1299);(2812,1300);(2878,1300);(2944,1301);(3010,1302);(3076,1303);(3142,1304);(3208,1305);(3274,1307);(3347,1308);(3418,1308);(3489,1306);(3560,1305);(3631,1307);(3702,1314);(3773,1315);(3844,1319);(3915,1323);(3986,1327);(4057,1334);(4128,1339);(4205,1344);(4274,1349);(4343,1354);(4412,1360);(4481,1366);(4550,1372);(4619,1379);(4688,1386);(4757,1393);(4826,1400);(4895,1408);(4964,1416)";
  String log_string = "(5564,1480);(5635,1489)";
  SerialDebug.println("Essaie : ");
  SerialDebug.println(checkCoordinates(ald, classicFP(), "Haut gauche du terrain, orienté vers la gauche", -40, 80, log_string));
}