/******************************************************************************
* Arduino sketch of a vehicle data data logger and telemeter for Freematics Hub
* Works with Freematics ONE+ Model A and Model B
* Developed by Stanley Huang <stanley@freematics.com.au>
* Distributed under BSD license
* Visit https://freematics.com/products for hardware information
* Visit https://hub.freematics.com to view live and history telemetry data
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
******************************************************************************/

#include <FreematicsPlus.h>
#include <httpd.h>
#include "config.h"
#include "mstedarls.h"
// #include "mptedarls.h"
#include "mptedarls_cpp.cpp"
#include "telestore.h"
#include "teleclient.h"
#include "telemesh.h"
#include <apps/sntp/sntp.h>
#include <SD.h>
#if BOARD_HAS_PSRAM
#include "esp32/himem.h"
#endif
#include "driver/adc.h"
#include "nvs_flash.h"
#include "nvs.h"
#if ENABLE_OLED
#include "FreematicsOLED.h"
#endif

#undef INFO
#define INFO FREEMATICS_INFO

#undef INFO
#define INFO 0x01

// states
#define STATE_STORAGE_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_READY 0x4
#define STATE_MEMS_READY 0x8
#define STATE_NET_READY 0x10
#define STATE_CELL_CONNECTED 0x20
#define STATE_WIFI_CONNECTED 0x40
#define STATE_WORKING 0x80
#define STATE_STANDBY 0x100

byte speed_mstedarls_flag = 0xA0;
byte rpm_mstedarls_flag = 0xA1;
byte tp_mstedarls_flag = 0xA2;
byte load_mstedarls_flag = 0xA3;
byte timing_mstedarls_flag = 0xA4;
byte speed_mptedarls_flag = 0xA5;
byte rpm_mptedarls_flag = 0xA6;
byte tp_mptedarls_flag = 0xA7;
byte load_mptedarls_flag = 0xA8;
byte timing_mptedarls_flag = 0xA9;

byte time_session_flag = 0xB8;

int count_model = 0;
int count_date = 0;
int count_auto_encoder = 0;
bool has_51 = false;
bool has_52 = false;
bool made_prediction = false;
bool time_compared = false;

typedef struct {
  byte pid;
  byte tier;
  int value;
  uint32_t ts;
} PID_POLLING_INFO;

PID_POLLING_INFO obdData[]= {
  {PID_ENGINE_LOAD, 1},
  {PID_RPM, 1},
  {PID_SPEED, 1},
  {PID_THROTTLE, 1},
  {PID_TIMING_ADVANCE, 1},
};

// const int len_input = 472;

// create a vector with 500 samples
// const int speed[len_input] = {15, 14, 15, 15, 17, 18, 20, 22, 24, 25, 26, 25, 20, 13, 9, 8, 8, 9, 11, 14, 17, 20, 23, 25, 26, 26, 26, 26, 25, 24, 22, 17, 11, 7, 3, 0, 0, 0, 2, 5, 6, 7, 8, 7, 7, 7, 6, 4, 0, 0, 4, 9, 14, 24, 34, 42, 51, 55, 54, 47, 36, 24, 18, 20, 28, 34, 42, 50, 56, 62, 64, 64, 63, 63, 66, 67, 69, 71, 71, 70, 68, 62, 59, 56, 55, 55, 54, 50, 42, 33, 23, 20, 22, 26, 31, 37, 44, 53, 59, 65, 70, 75, 80, 85, 89, 94, 96, 98, 99, 98, 92, 88, 86, 85, 79, 72, 66, 63, 59, 52, 43, 35, 28, 26, 27, 29, 31, 32, 32, 32, 32, 33, 33, 34, 35, 35, 35, 32, 26, 24, 22, 21, 20, 19, 19, 19, 18, 13, 5, 0, 0, 0, 0, 3, 5, 8, 10, 13, 17, 19, 23, 26, 30, 33, 37, 41, 44, 48, 51, 54, 56, 59, 60, 62, 61, 57, 51, 44, 37, 28, 19, 18, 20, 23, 27, 30, 33, 33, 30, 24, 17, 14, 16, 19, 23, 27, 31, 35, 39, 42, 43, 43, 41, 38, 38, 37, 38, 39, 40, 33, 24, 18, 17, 19, 27, 34, 40, 48, 55, 62, 67, 69, 69, 67, 61, 57, 54, 53, 50, 44, 39, 37, 37, 38, 39, 40, 39, 37, 35, 35, 36, 37, 37, 38, 39, 39, 40, 41, 42, 43, 44, 44, 44, 45, 45, 45, 46, 45, 44, 43, 42, 41, 38, 36, 34, 32, 31, 30, 30, 30, 29, 28, 28, 28, 29, 30, 31, 32, 34, 35, 37, 38, 38, 40, 40, 40, 41, 42, 43, 44, 45, 45, 45, 43, 39, 34, 28, 25, 23, 23, 25, 33, 39, 42, 44, 40, 38, 37, 36, 36, 35, 35, 35, 35, 35, 34, 34, 34, 34, 33, 34, 34, 35, 35, 32, 28, 23, 17, 15, 17, 21, 25, 28, 29, 30, 30, 28, 27, 27, 27, 29, 30, 31, 33, 33, 33, 32, 32, 31, 31, 30, 29, 29, 28, 27, 26, 25, 25, 24, 24, 23, 22, 20, 20, 20, 20, 20, 19, 18, 17, 15, 13, 9, 3, 2, 3, 3, 4, 5, 6, 6, 6, 6, 5, 7, 13, 19, 24, 26, 29, 34, 37, 39, 40, 41, 44, 47, 51, 53, 55, 56, 57, 57, 58, 58, 60, 61, 62, 59, 57, 56, 56, 59, 65, 68, 72, 73, 72, 69, 68, 67, 65, 63, 62, 60, 53, 38, 24, 19, 19, 20, 24, 29, 34, 38, 43, 49, 55, 60, 65, 70, 75, 79, 83, 87, 91, 94, 96, 97, 96, 94, 89, 82, 78, 71, 64, 53, 41, 29, 18, 16, 17, 20, 24, 28, 32, 37, 42, 47, 52, 57, 60, 64, 68, 71, 72, 72, 68, 58, 45, 30, 18, 16, 18, 20, 24, 28, 31, 34, 37, 40, 43, 47, 50, 54, 57, 60, 63, 65, 64, 61, 54, 43, 28, 17, 13, 14, 19, 23, 28, 33, 38, 43, 48, 53, 57, 59, 61, 62, 64, 67, 70, 72, 75, 76, 76, 75, 73, 67, 64, 62, 60, 57, 49, 40, 31, 26, 28, 31, 34, 37, 40, 41, 42, 41, 38, 34, 27, 20, 13, 8, 5, 5, 5, 6, 8, 9, 11, 13, 18, 23, 27, 30, 33, 35, 36, 35, 29, 22, 17, 14, 11, 11, 12, 16, 20, 23, 27, 27, 27, 25, 23, 19, 14, 11, 10, 11, 14, 17, 19, 18, 15, 9, 5, 3, 0, 0, 0, 0, 0, 0};
// const int rpm[len_input] = {1485, 1531, 1618, 1595, 1538, 1502, 1550, 1467, 1173, 1101, 1099, 1083, 1097, 1054, 967, 936, 942, 1168, 1517, 1864, 1810, 1859, 1560, 1517, 1506, 1592, 1513, 1197, 1120, 1091, 1089, 1079, 1020, 936, 1006, 928, 947, 915, 963, 1001, 928, 923, 1055, 988, 975, 954, 964, 1008, 981, 973, 1181, 1646, 2158, 2804, 2513, 2531, 2612, 2141, 1610, 1400, 1510, 1345, 1190, 2386, 2422, 2197, 2463, 2622, 2649, 2610, 1967, 1702, 1530, 2317, 2416, 2101, 2332, 2356, 1863, 1631, 1523, 1563, 1569, 1515, 1525, 1536, 1517, 1486, 1516, 1510, 1256, 1288, 1791, 2053, 1996, 2243, 2445, 2639, 2498, 2661, 2700, 2668, 2686, 2770, 2914, 2877, 2362, 2694, 2475, 1886, 1782, 1697, 1742, 1649, 1558, 1603, 1586, 1561, 1511, 1507, 1495, 1541, 1505, 1605, 1824, 1802, 1582, 1490, 1515, 1519, 1531, 1530, 1515, 1519, 1524, 1518, 1530, 1483, 1259, 1178, 1109, 1098, 1094, 1101, 1542, 1492, 1364, 1095, 879, 984, 968, 955, 953, 996, 1013, 1280, 1462, 1862, 1894, 1909, 1915, 1737, 1854, 1903, 2002, 2037, 2142, 2232, 2146, 2143, 2153, 1998, 1829, 1456, 1531, 1529, 1526, 1512, 1514, 1479, 1161, 1600, 1827, 1822, 1683, 1581, 1500, 1509, 1482, 1144, 1142, 1796, 2167, 2183, 2284, 2007, 2139, 2188, 2191, 2208, 1766, 1402, 1534, 1547, 1533, 1523, 1517, 1554, 1508, 1454, 1253, 1150, 1287, 2133, 2427, 1745, 2058, 2325, 2612, 2460, 2132, 1778, 1470, 1596, 1529, 1534, 1536, 1513, 1513, 1487, 1583, 1543, 1547, 1584, 1583, 1517, 1526, 1509, 1509, 1565, 1534, 1525, 1523, 1534, 1522, 1534, 1526, 1542, 1522, 1521, 1514, 1516, 1512, 1523, 1527, 1526, 1544, 1501, 1532, 1546, 1531, 1520, 1504, 1519, 1524, 1527, 1530, 1525, 1525, 1523, 1506, 1646, 1664, 1681, 1535, 1528, 1541, 1522, 1530, 1530, 1545, 1517, 1500, 1534, 1528, 1519, 1529, 1531, 1535, 1500, 1513, 1547, 1482, 1538, 1504, 1509, 1506, 1186, 1127, 1762, 2276, 2453, 2436, 1793, 1438, 1522, 1524, 1548, 1514, 1516, 1518, 1525, 1520, 1517, 1519, 1530, 1527, 1523, 1535, 1522, 1530, 1526, 1512, 1524, 1484, 1488, 1178, 1135, 1397, 2065, 2019, 1847, 1532, 1509, 1509, 1500, 1489, 1600, 1718, 1683, 1565, 1529, 1530, 1518, 1512, 1520, 1527, 1524, 1511, 1545, 1528, 1508, 1517, 1519, 1522, 1538, 1511, 1537, 1534, 1524, 1496, 1522, 1519, 1550, 1512, 1499, 1518, 1501, 1487, 1614, 1580, 1458, 1223, 1003, 1007, 977, 999, 968, 1304, 1125, 1002, 1022, 981, 988, 1436, 2292, 2305, 2143, 1990, 2155, 2358, 2253, 2172, 2284, 2204, 2477, 2338, 2577, 2327, 2296, 2278, 1817, 2165, 1855, 1869, 1955, 1416, 1585, 1505, 1541, 1528, 1628, 2355, 2401, 2244, 2372, 1840, 1501, 1614, 1587, 1551, 1550, 1565, 1555, 1528, 1449, 1569, 1243, 1138, 1815, 1982, 1900, 1960, 2025, 2215, 2200, 2389, 2465, 2509, 2576, 2512, 2544, 2544, 2528, 2434, 2624, 2282, 2146, 1897, 1845, 1855, 1751, 1608, 1631, 1565, 1553, 1499, 1515, 1485, 1159, 1530, 1909, 2049, 1996, 1911, 1976, 2223, 2266, 2296, 2316, 2277, 2293, 2331, 2280, 2301, 1785, 1510, 1566, 1538, 1491, 1426, 1125, 1534, 1843, 1994, 1888, 1843, 1877, 1947, 2098, 2048, 2150, 2279, 2249, 2521, 2447, 2482, 2515, 2380, 1776, 1434, 1519, 1503, 1262, 1131, 1477, 2072, 2444, 2264, 2159, 2230, 2279, 2298, 2294, 2215, 1996, 1802, 1472, 1556, 1788, 1978, 1910, 2032, 1998, 1595, 1594, 1648, 1575, 1548, 1585, 1542, 1528, 1511, 1509, 1515, 1528, 1148, 1729, 1796, 1672, 1647, 1664, 1547, 1502, 1503, 1514, 1480, 1415, 1143, 1093, 993, 994, 1007, 991, 1171, 1018, 1243, 1592, 2002, 2224, 2109, 1822, 1808, 1793, 1698, 1466, 1516, 1471, 1180, 1139, 1054, 1024, 1649, 1759, 1929, 1971, 1975, 1729, 1780, 1875, 1625, 1593, 1548, 1345, 1040, 1154, 1644, 1928, 1830, 1608, 1496, 1409, 1155, 966, 1029, 991, 983, 1015, 971, 961, 0};
// const int tp[len_input] = {14, 16, 17, 20, 20, 18, 15, 15, 15, 14, 14, 14, 14, 14, 14, 14, 14, 16, 21, 21, 21, 20, 19, 17, 15, 14, 14, 14, 14, 14, 14, 14, 14, 15, 18, 17, 17, 16, 18, 18, 16, 15, 14, 16, 17, 16, 18, 20, 18, 18, 21, 89, 89, 89, 88, 89, 32, 39, 15, 14, 16, 15, 17, 89, 89, 89, 89, 39, 51, 36, 89, 89, 25, 89, 28, 84, 89, 29, 31, 16, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 16, 16, 23, 89, 89, 89, 89, 32, 81, 38, 41, 40, 40, 42, 89, 34, 38, 89, 17, 12, 12, 13, 23, 12, 12, 14, 14, 14, 13, 14, 14, 14, 14, 19, 23, 25, 28, 18, 14, 16, 16, 21, 22, 20, 20, 16, 14, 14, 15, 15, 14, 15, 15, 15, 15, 15, 17, 15, 20, 19, 19, 14, 20, 19, 20, 20, 23, 26, 27, 32, 89, 89, 89, 89, 89, 89, 89, 89, 89, 89, 89, 32, 22, 14, 12, 14, 14, 14, 14, 14, 15, 20, 23, 32, 89, 29, 21, 13, 13, 17, 15, 24, 59, 89, 89, 89, 89, 89, 89, 89, 27, 13, 12, 16, 18, 20, 22, 24, 13, 12, 16, 14, 16, 80, 89, 24, 89, 85, 89, 33, 32, 27, 14, 13, 14, 14, 14, 14, 14, 14, 14, 14, 23, 29, 29, 14, 14, 13, 14, 21, 23, 21, 20, 21, 21, 21, 21, 22, 21, 21, 19, 17, 15, 15, 14, 14, 13, 12, 14, 14, 14, 14, 14, 14, 14, 14, 16, 15, 14, 14, 14, 17, 18, 20, 21, 22, 22, 23, 23, 24, 27, 22, 22, 21, 20, 20, 21, 20, 19, 13, 14, 13, 12, 14, 14, 14, 14, 15, 14, 22, 89, 32, 89, 30, 12, 12, 14, 16, 15, 16, 18, 17, 17, 17, 15, 14, 13, 14, 13, 15, 19, 21, 14, 14, 13, 14, 15, 14, 20, 29, 83, 89, 24, 20, 13, 14, 14, 18, 18, 20, 22, 23, 22, 21, 20, 14, 15, 15, 16, 20, 16, 16, 16, 18, 18, 20, 18, 21, 22, 21, 15, 16, 16, 21, 21, 21, 20, 20, 13, 15, 13, 12, 16, 19, 18, 18, 18, 18, 20, 15, 15, 17, 18, 19, 29, 89, 89, 89, 89, 89, 89, 89, 32, 89, 89, 89, 65, 36, 30, 32, 89, 19, 33, 23, 23, 25, 23, 12, 12, 13, 13, 30, 89, 31, 30, 24, 22, 11, 13, 13, 12, 12, 12, 12, 12, 13, 13, 15, 14, 20, 32, 89, 89, 89, 89, 89, 89, 89, 89, 89, 35, 89, 30, 88, 39, 59, 32, 27, 17, 12, 12, 12, 12, 12, 12, 12, 13, 13, 12, 14, 18, 25, 33, 89, 89, 89, 89, 89, 89, 89, 28, 84, 89, 89, 33, 32, 17, 12, 13, 13, 13, 14, 19, 23, 28, 89, 89, 89, 89, 89, 89, 89, 89, 89, 89, 89, 89, 89, 26, 16, 13, 13, 13, 14, 14, 21, 88, 89, 89, 89, 89, 89, 89, 34, 89, 30, 21, 13, 18, 89, 89, 89, 89, 89, 17, 22, 11, 12, 13, 13, 12, 12, 12, 13, 13, 13, 15, 21, 24, 30, 89, 89, 22, 12, 12, 14, 14, 14, 15, 14, 14, 18, 18, 18, 16, 13, 20, 23, 89, 89, 89, 89, 89, 89, 89, 14, 14, 13, 16, 15, 14, 14, 25, 27, 25, 89, 89, 89, 89, 31, 18, 14, 14, 16, 15, 16, 23, 26, 24, 14, 14, 14, 15, 18, 18, 18, 17, 16, 14, 14, 10};
// const int load[len_input] = {25, 24, 25, 31, 37, 31, 23, 25, 36, 35, 34, 32, 32, 32, 34, 35, 35, 36, 40, 32, 38, 36, 44, 29, 26, 20, 20, 36, 34, 32, 33, 33, 33, 35, 58, 58, 58, 40, 60, 60, 55, 47, 38, 36, 50, 49, 55, 63, 64, 63, 61, 52, 50, 75, 84, 89, 99, 72, 20, 31, 29, 33, 42, 45, 60, 72, 81, 93, 95, 92, 52, 69, 45, 65, 63, 67, 68, 64, 56, 35, 19, 18, 20, 21, 21, 22, 22, 22, 22, 22, 31, 36, 37, 43, 58, 70, 79, 92, 78, 96, 94, 92, 99, 96, 100, 100, 65, 87, 26, 10, 10, 13, 37, 16, 13, 16, 18, 19, 20, 21, 23, 21, 22, 33, 31, 39, 52, 35, 23, 23, 25, 35, 40, 34, 32, 29, 26, 23, 27, 38, 34, 33, 34, 35, 22, 24, 40, 38, 40, 63, 64, 58, 60, 65, 67, 47, 46, 39, 39, 42, 43, 61, 60, 59, 56, 58, 60, 64, 63, 61, 60, 55, 52, 24, 21, 21, 23, 23, 23, 24, 38, 27, 35, 46, 55, 58, 50, 21, 24, 31, 36, 35, 41, 45, 51, 62, 64, 62, 63, 63, 58, 25, 20, 21, 29, 33, 39, 47, 23, 22, 30, 36, 32, 45, 56, 57, 74, 71, 96, 81, 64, 53, 23, 19, 20, 21, 20, 21, 21, 23, 20, 22, 43, 51, 52, 27, 19, 22, 24, 32, 48, 36, 35, 37, 34, 38, 37, 40, 37, 35, 30, 29, 25, 25, 21, 21, 18, 18, 21, 22, 22, 22, 23, 23, 23, 24, 22, 24, 20, 20, 20, 22, 23, 27, 34, 39, 37, 45, 49, 52, 50, 49, 41, 38, 33, 33, 36, 32, 31, 25, 22, 19, 20, 20, 22, 22, 24, 37, 35, 31, 51, 69, 75, 53, 31, 23, 22, 23, 22, 26, 32, 31, 30, 30, 26, 24, 21, 21, 20, 24, 29, 33, 29, 20, 23, 23, 36, 34, 33, 37, 39, 49, 58, 40, 25, 20, 20, 26, 23, 28, 38, 43, 43, 40, 33, 23, 23, 21, 23, 29, 27, 27, 25, 29, 29, 32, 29, 32, 36, 38, 26, 24, 25, 34, 40, 38, 34, 33, 25, 22, 20, 21, 31, 57, 59, 60, 59, 60, 53, 42, 39, 51, 55, 60, 61, 56, 49, 52, 66, 58, 58, 55, 47, 49, 47, 63, 64, 81, 65, 51, 50, 23, 40, 36, 31, 34, 60, 18, 16, 18, 20, 38, 63, 68, 60, 67, 58, 15, 14, 16, 17, 16, 17, 16, 18, 21, 20, 27, 34, 26, 40, 51, 55, 59, 59, 62, 74, 80, 80, 85, 78, 78, 73, 77, 63, 78, 56, 45, 47, 10, 10, 12, 13, 13, 15, 16, 18, 18, 19, 34, 27, 34, 39, 45, 56, 57, 60, 66, 69, 70, 67, 65, 67, 67, 62, 59, 51, 15, 16, 19, 19, 32, 28, 32, 35, 44, 56, 54, 57, 54, 54, 61, 63, 64, 78, 82, 81, 81, 76, 39, 18, 17, 21, 22, 32, 38, 46, 44, 53, 63, 67, 66, 69, 69, 63, 55, 53, 20, 27, 60, 52, 50, 51, 52, 40, 40, 14, 12, 16, 16, 16, 17, 17, 19, 19, 19, 33, 28, 34, 49, 53, 52, 52, 20, 20, 21, 22, 23, 37, 35, 33, 52, 60, 59, 41, 34, 42, 45, 44, 43, 49, 61, 61, 61, 58, 43, 23, 26, 36, 38, 33, 33, 49, 43, 35, 47, 49, 60, 69, 51, 34, 24, 22, 38, 37, 39, 41, 36, 38, 25, 24, 25, 39, 45, 60, 56, 58, 47, 38, 35, 41};
// const int timing[len_input] = {24, 23, 27, 21, 18, 29, 26, -1, -29, -19, -20, -16, -20, -16, -12, -11, -10, 0, 16, 13, 12, 17, 17, 27, 24, 30, -11, -24, -23, -19, -21, -19, -15, -3, -12, -9, -9, -1, -9, -11, -9, -11, 10, -2, -14, -13, -11, -12, -12, -10, 10, 13, 8, 10, 1, 0, 1, 19, -26, 0, 0, 0, 11, 9, 8, 5, 1, 0, -1, 1, 12, 31, 11, 5, 12, 8, 7, 14, 12, 33, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -20, -4, -2, 7, 4, 3, 3, 2, 2, 0, 0, 1, 0, 0, 0, 0, 5, 1, 42, 0, 0, 0, 11, 21, 0, 0, 0, 0, 0, 0, 0, 0, 0, 21, 20, 18, 13, 31, 29, 25, 23, 22, 23, 28, 29, 29, 30, 0, 0, -22, -20, -19, -17, 7, 20, 3, -32, -21, 11, -10, -11, -5, -15, -11, 5, 12, 14, 14, 12, 15, 12, 11, 11, 10, 10, 11, 8, 9, 10, 10, 11, 12, 31, 25, 0, 0, 0, 0, 0, 0, -24, 0, 9, 7, 12, 13, 22, 6, 0, -34, -22, 0, 9, 11, 11, 10, 10, 10, 10, 10, 12, 26, 0, 17, 27, 25, 22, 12, 32, 0, -32, -26, -11, -1, 8, 12, 6, 5, -1, 3, 8, 13, -17, 0, 0, 0, 0, 0, 0, 0, 0, 0, 12, 13, 13, 29, 3, 0, 0, 18, 19, 25, 25, 25, 25, 24, 25, 23, 25, 26, 28, 30, 26, 24, 29, 27, 11, 0, 0, 0, 0, 0, 0, 0, 0, 0, 18, 29, 30, 15, 0, 28, 26, 28, 25, 22, 23, 20, 18, 18, 13, 14, 24, 24, 28, 28, 26, 29, 30, 29, 28, 29, 0, 0, 0, 0, 0, -31, -27, 0, -3, 5, 5, 11, 28, 0, 0, 29, 26, 25, 26, 28, 27, 29, 26, 28, 28, 28, 28, 24, 29, 28, 31, -2, 0, 0, -29, -28, -6, 10, 15, 12, 13, 21, 31, 8, 0, 21, 26, 26, 23, 22, 22, 24, 30, 28, 25, 28, 23, 27, 29, 25, 26, 24, 28, 28, 28, 26, 23, 27, 26, 24, 24, 19, 21, 24, 25, 28, 29, 27, 32, 14, -18, -14, -13, -12, -12, -14, 10, 9, 7, -13, -12, 6, 12, 9, 11, 13, 11, 9, 12, 13, 15, 13, 9, 7, 4, 7, 11, 14, 15, 11, 16, 14, 15, 14, 13, 35, 0, 0, 0, 11, 5, 7, 7, 8, 33, -21, 0, 0, 0, 0, 0, 0, 0, 0, 0, -36, -25, 16, 7, 8, 9, 10, 11, 10, 5, 4, 3, 2, 3, 3, 1, 3, 5, 12, 8, 15, 37, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -23, -3, 8, 8, 11, 7, 9, 9, 7, 8, 8, 9, 8, 8, 8, 15, 12, 34, 0, 0, 0, 0, -30, -2, 10, 9, 12, 12, 13, 11, 12, 12, 9, 9, 8, 4, 3, 3, 3, 21, 36, 0, 0, 0, 18, -23, -6, 4, 15, 10, 11, 8, 8, 8, 8, 10, 13, 33, 0, 20, 10, 13, 11, 13, 16, 27, 20, 16, 0, 0, 0, 0, 0, 0, 0, 0, 0, -32, 19, 18, 13, 13, 13, 14, 30, 0, 0, 0, 0, -30, -23, -15, -13, -13, -4, 16, 9, 10, 15, 12, 15, 14, 11, 11, 11, 13, 28, -11, 0, -22, -25, -23, -21, 3, 11, 13, 12, 12, 13, 12, 14, 21, 27, 2, -14, -16, 7, 12, 15, 20, 32, 15, -6, -12, -10, -14, -13, -13, -20, -9, -9, 0};

// const int speed[len_input] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 3, 3, 4, 5, 4, 3, 2, 0, 0, 1, 2, 3, 2, 2, 2, 2, 1, 0, 0, 1, 5, 8, 10, 11, 12, 13, 15, 15, 15, 14, 14, 14, 13, 13, 13, 13, 14, 16, 18, 19, 19, 20, 20, 17, 14, 12, 10, 11, 12, 12, 14, 15, 17, 18, 19, 20, 20, 21, 21, 20, 18, 17, 16, 15, 13, 10, 5, 2, 1, 0, 3, 5, 7, 9, 8, 7, 6, 6, 7, 8, 10, 12, 14, 16, 18, 20, 22, 24, 25, 25, 25, 24, 23, 22, 21, 20, 19, 17, 16, 16, 17, 19, 21, 22, 25, 27, 28, 30, 30, 32, 32, 33, 34, 34, 35, 35, 36, 36, 36, 37, 37, 37, 38, 38, 38, 38, 38, 38, 39, 39, 40, 41, 40, 39, 38, 38, 38, 37, 37, 36, 33, 30, 25, 19, 17, 15, 16, 18, 19, 21, 23, 24, 25, 25, 27, 29, 30, 31, 33, 34, 36, 37, 39, 40, 42, 44, 47, 49, 51, 52, 54, 55, 56, 57, 57, 58, 59, 60, 61, 61, 60, 58, 55, 53, 49, 45, 41, 37, 35, 35, 34, 33, 32, 31, 30, 28, 28, 28, 27, 27, 27, 27, 27, 27, 28, 28, 28, 28, 29, 29, 29, 29, 28, 28, 29, 29, 30, 31, 31, 32, 33, 34, 35, 36, 37, 38, 40, 42, 44, 47, 49, 51, 53, 55, 56, 58, 58, 60, 61, 61, 62, 62, 62, 62, 63, 63, 61, 59, 57, 54, 51, 47, 40, 34, 28, 21, 17, 15, 18, 20, 22, 24, 26, 29, 30, 31, 32, 34, 35, 36, 37, 38, 39, 39, 40, 40, 42, 43, 44, 45, 47, 49, 51, 52, 54, 55, 56, 56, 56, 56, 55, 53, 49, 43, 36, 29, 23, 18, 17, 20, 24, 27, 30, 32, 35, 37, 39, 40, 41, 42, 43, 45, 46, 47, 48, 49, 48, 47, 45, 43, 41, 38, 32, 26, 22, 17, 19, 21, 22, 24, 25, 27, 28, 29, 31, 31, 32, 33, 33, 34, 35, 34, 34, 33, 32, 31, 30, 28, 24, 20, 19, 18, 18, 19, 20, 22, 23, 25, 26, 28, 28, 29, 30, 32, 33, 35, 35, 34, 34, 33, 31, 30, 29, 30, 31, 32, 32, 34, 32, 31, 30, 27, 24, 21, 21, 20, 19, 16, 13, 11, 10, 10, 8, 8, 9, 10, 11, 12, 14, 16, 18, 20, 20, 21, 22, 22, 22, 21, 20, 18, 17, 15, 14, 12, 12, 12, 13, 16, 18, 19, 20, 19, 19, 18, 17, 16, 14, 13, 12, 11, 12, 12, 13, 13, 14, 13, 12, 11, 10, 10, 9, 8, 7, 5, 3, 1, 0, 0, 1, 2, 3, 2, 2, 1, 0, 0};
// const int rpm[len_input] = {982, 981, 977, 977, 974, 978, 979, 983, 981, 977, 980, 984, 934, 980, 975, 946, 924, 926, 889, 938, 880, 898, 909, 914, 960, 971, 967, 921, 927, 958, 950, 972, 918, 965, 942, 937, 1047, 1381, 1541, 1713, 1911, 2039, 2142, 1986, 1259, 1007, 924, 1255, 1453, 1493, 1219, 987, 931, 917, 982, 939, 937, 933, 912, 873, 858, 846, 835, 852, 868, 994, 1281, 1424, 1483, 1574, 1618, 1665, 1685, 1668, 1639, 1323, 1126, 989, 924, 893, 872, 841, 778, 897, 870, 948, 945, 906, 927, 882, 922, 862, 883, 888, 892, 909, 1047, 1225, 1350, 1515, 1663, 1809, 1930, 2036, 2109, 2063, 2051, 1942, 1541, 1496, 1361, 1276, 1084, 950, 919, 1286, 1543, 1721, 1875, 1983, 2186, 2311, 1762, 1660, 1593, 1583, 1638, 1678, 1711, 1742, 1764, 1767, 1754, 1775, 1788, 1811, 1828, 1843, 1856, 1861, 1862, 1869, 1866, 1876, 1897, 1920, 1959, 1484, 1444, 1424, 1395, 1367, 1348, 1347, 1337, 1300, 1621, 1470, 1243, 1168, 950, 913, 1412, 1551, 1676, 1788, 1913, 2000, 2068, 2140, 2304, 2134, 1620, 1577, 1647, 1676, 1753, 1823, 1896, 1511, 1581, 1643, 1718, 1759, 1832, 1415, 1485, 1527, 1541, 1571, 1580, 1616, 1633, 1665, 1674, 1672, 1591, 1540, 1487, 1419, 1456, 1621, 1817, 1830, 1737, 1691, 1669, 1636, 1593, 1532, 1458, 1396, 1403, 1376, 1354, 1327, 1312, 1320, 1334, 1351, 1359, 1374, 1382, 1392, 1409, 1419, 1413, 1408, 1404, 1407, 1423, 1449, 1462, 1499, 1527, 1574, 1632, 1673, 1723, 1770, 1814, 1872, 1451, 1573, 1646, 1712, 1761, 1835, 1906, 1526, 1555, 1579, 1609, 1634, 1670, 1686, 1697, 1705, 1707, 1716, 1717, 1670, 1640, 1589, 1529, 1456, 1371, 1256, 1296, 1245, 1111, 985, 964, 1416, 1650, 1834, 1988, 2139, 2323, 2440, 1715, 1671, 1626, 1695, 1756, 1789, 1830, 1860, 1895, 1917, 1943, 1982, 2039, 1546, 1635, 1679, 1729, 1764, 1819, 1869, 1489, 1530, 1505, 1495, 1498, 1489, 1473, 1417, 1329, 1192, 1305, 1225, 1081, 1009, 1252, 1780, 2043, 2299, 1790, 1784, 1797, 1828, 1907, 1509, 1541, 1571, 1608, 1651, 1695, 1726, 1739, 1747, 1741, 1700, 1633, 1562, 1496, 1374, 1172, 1297, 1082, 1468, 1701, 1861, 1976, 2114, 2213, 2310, 2404, 2499, 2585, 2616, 2293, 1794, 1745, 1693, 1705, 1712, 1639, 1626, 1566, 1520, 1463, 1376, 1217, 1233, 1178, 1447, 1616, 1710, 1806, 1927, 2024, 2148, 2239, 2334, 2404, 1672, 1639, 1590, 1671, 1721, 1715, 1675, 1660, 1638, 1553, 1456, 1442, 1455, 1496, 1551, 1596, 1606, 1591, 1539, 1478, 1314, 1215, 1531, 1543, 1387, 1208, 1023, 907, 854, 828, 842, 831, 847, 877, 1070, 1211, 1270, 1394, 1534, 1628, 1717, 1760, 1817, 1850, 1864, 1858, 1645, 1507, 1372, 1343, 1248, 1049, 1061, 1202, 1243, 1351, 1510, 1600, 1717, 1807, 1748, 1732, 1673, 1611, 1490, 1192, 972, 896, 858, 1111, 1145, 1169, 1207, 1183, 966, 888, 859, 847, 837, 832, 846, 865, 867, 926, 966, 950, 942, 972, 960, 920, 934, 943, 936, 981, 1023};
// const int tp[len_input] = {15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 19, 20, 18, 19, 83, 82, 21, 19, 18, 18, 19, 21, 19, 19, 21, 17, 19, 23, 32, 29, 51, 18, 17, 19, 20, 21, 19, 18, 20, 20, 20, 19, 14, 13, 14, 13, 16, 16, 16, 14, 14, 13, 14, 12, 12, 12, 12, 11, 13, 14, 14, 14, 14, 14, 19, 18, 19, 18, 18, 17, 17, 16, 16, 15, 12, 12, 12, 13, 12, 13, 14, 16, 16, 19, 18, 20, 30, 20, 18, 15, 18, 19, 20, 20, 82, 21, 20, 21, 22, 23, 22, 22, 22, 20, 18, 18, 14, 12, 13, 12, 12, 12, 12, 12, 20, 21, 23, 23, 24, 25, 24, 23, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 24, 25, 25, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 22, 14, 13, 12, 12, 12, 12, 12, 12, 13, 12, 12, 12, 12, 13, 17, 18, 17, 18, 18, 18, 17, 19, 21, 19, 20, 20, 21, 21, 21, 22, 23, 25, 30, 32, 32, 31, 29, 29, 41, 30, 28, 28, 29, 29, 30, 30, 28, 18, 13, 13, 12, 12, 12, 13, 13, 13, 16, 15, 16, 14, 13, 12, 12, 13, 13, 14, 15, 14, 14, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 16, 17, 17, 17, 17, 19, 20, 20, 19, 19, 19, 19, 24, 23, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 21, 23, 23, 23, 23, 15, 13, 13, 12, 12, 12, 12, 12, 12, 12, 14, 11, 21, 22, 25, 25, 26, 27, 27, 26, 29, 29, 29, 29, 29, 28, 27, 27, 27, 27, 26, 26, 25, 29, 30, 30, 27, 23, 25, 25, 24, 15, 13, 13, 13, 12, 12, 12, 12, 12, 12, 15, 12, 21, 21, 23, 24, 23, 25, 26, 26, 26, 27, 27, 26, 26, 26, 25, 25, 25, 24, 14, 13, 13, 12, 12, 12, 13, 12, 11, 20, 22, 24, 25, 25, 25, 24, 25, 25, 23, 22, 23, 22, 21, 20, 17, 13, 12, 13, 12, 12, 12, 12, 12, 12, 12, 17, 19, 20, 21, 21, 21, 21, 20, 21, 22, 23, 23, 23, 22, 21, 13, 13, 13, 12, 12, 12, 12, 13, 16, 16, 16, 12, 13, 12, 12, 12, 12, 13, 13, 12, 12, 12, 12, 13, 15, 14, 15, 15, 14, 17, 20, 20, 21, 20, 19, 18, 18, 17, 17, 17, 17, 13, 13, 12, 14, 14, 13, 16, 18, 18, 21, 20, 20, 20, 20, 21, 21, 21, 21, 17, 13, 13, 13, 14, 16, 16, 16, 16, 14, 13, 12, 12, 13, 14, 14, 15, 16, 19, 20, 20, 14, 17, 17, 15, 18, 20, 19, 16, 17, 10};
// const int load[len_input] = {16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 26, 27, 21, 34, 34, 32, 28, 22, 24, 27, 29, 27, 28, 31, 26, 24, 30, 34, 34, 34, 28, 27, 25, 27, 33, 27, 22, 23, 23, 21, 20, 11, 7, 11, 12, 22, 16, 16, 12, 12, 14, 13, 13, 9, 8, 7, 7, 7, 12, 13, 14, 15, 13, 17, 25, 25, 21, 20, 18, 16, 16, 14, 12, 7, 8, 8, 9, 9, 10, 12, 19, 18, 21, 26, 27, 32, 30, 25, 19, 22, 26, 28, 28, 33, 31, 28, 27, 29, 28, 28, 26, 25, 21, 15, 15, 14, 8, 7, 6, 6, 6, 7, 7, 24, 28, 29, 29, 28, 27, 27, 26, 32, 33, 34, 33, 33, 33, 33, 32, 33, 32, 32, 32, 32, 31, 30, 30, 31, 30, 31, 30, 30, 30, 30, 29, 29, 17, 7, 6, 6, 6, 6, 6, 6, 6, 7, 6, 8, 7, 8, 21, 18, 19, 18, 19, 18, 17, 20, 22, 20, 25, 23, 27, 27, 26, 27, 28, 33, 40, 43, 42, 40, 36, 38, 49, 45, 41, 38, 38, 40, 40, 40, 39, 23, 7, 6, 6, 6, 6, 7, 7, 7, 10, 12, 12, 12, 7, 7, 7, 7, 8, 9, 10, 12, 12, 17, 16, 16, 16, 17, 16, 16, 16, 16, 16, 16, 17, 18, 20, 21, 21, 21, 22, 25, 25, 24, 23, 22, 22, 27, 30, 32, 33, 32, 32, 32, 32, 34, 34, 34, 34, 34, 34, 34, 26, 28, 29, 30, 29, 23, 7, 7, 7, 6, 5, 6, 7, 6, 6, 8, 7, 29, 29, 32, 33, 32, 33, 32, 36, 36, 38, 36, 36, 36, 35, 33, 32, 32, 31, 30, 30, 32, 38, 38, 38, 34, 32, 29, 34, 32, 21, 7, 7, 6, 6, 6, 6, 6, 6, 6, 8, 6, 12, 28, 28, 28, 27, 32, 33, 34, 35, 36, 37, 36, 35, 35, 34, 34, 34, 33, 17, 7, 7, 7, 7, 6, 6, 6, 7, 22, 27, 30, 31, 31, 29, 28, 29, 29, 26, 21, 25, 27, 27, 26, 18, 7, 7, 6, 7, 7, 7, 6, 5, 7, 6, 17, 23, 23, 25, 25, 25, 24, 21, 23, 23, 29, 30, 30, 30, 27, 10, 7, 7, 7, 7, 6, 6, 5, 12, 15, 14, 11, 7, 7, 7, 6, 6, 7, 7, 6, 6, 6, 8, 11, 12, 13, 15, 18, 16, 25, 26, 25, 28, 25, 23, 21, 20, 18, 18, 17, 15, 9, 8, 7, 11, 9, 9, 23, 26, 22, 27, 25, 24, 24, 25, 26, 26, 26, 26, 24, 10, 9, 9, 10, 19, 17, 17, 17, 16, 9, 10, 10, 10, 12, 16, 19, 21, 25, 26, 27, 26, 21, 26, 22, 24, 27, 27, 27, 25, 14};
// const int timing[len_input] = {4, 5, 4, 5, 6, 6, 4, 6, 3, 4, 6, -18, -15, -14, -18, -17, -16, -12, -15, -13, -10, -11, -10, -19, -12, -15, -8, -9, -13, -15, -16, -16, -9, -3, -23, -17, -16, 6, 15, 17, 18, 19, 20, 10, 19, 2, -5, -5, 15, 17, 5, 2, -1, -3, -8, -6, -3, -4, -3, -3, -7, -5, -6, -9, -5, -13, 5, 14, 16, 16, 17, 18, 15, 14, 12, 9, 1, -4, -3, -1, 3, -9, -11, -9, -10, -7, -12, -22, -19, -16, -7, -12, -11, -12, -13, -18, -12, 10, 11, 14, 15, 16, 17, 18, 20, 12, 12, 3, -9, 30, 7, 2, -1, -3, 2, -18, 6, 13, 16, 16, 17, 18, 14, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 13, -2, -8, -9, -9, -9, -9, -9, -9, -7, -9, 18, 0, -7, 14, -3, 15, 17, 18, 18, 19, 18, 20, 20, 11, 15, 15, 15, 15, 16, 16, 15, 12, 8, 7, 8, 9, 11, 6, 5, 7, 9, 11, 11, 9, 9, 9, 9, 0, -1, -7, -9, -9, -9, -7, -4, -3, 3, 13, 14, 0, -7, -9, -9, 31, 28, 25, 8, 20, 17, 12, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 15, 15, 15, 16, 17, 18, 17, 15, 12, 13, 13, 13, 13, 14, -7, 12, 12, 12, 12, 12, 13, 13, 15, 15, 15, 15, 15, -5, -6, -7, -9, -5, -6, -3, -9, -9, -9, -4, -3, -6, 12, 14, 14, 15, 15, -7, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 11, 9, 9, 9, 12, 15, -6, 12, 13, -3, -6, -9, -9, -9, -9, -9, -9, -9, -9, -8, -5, -22, 15, 15, 17, 15, 14, 13, 13, -1, 12, 12, 12, 12, 12, 12, 12, 13, 12, 6, -5, -6, -8, -9, -8, 7, -9, -9, -18, 13, 15, 15, 15, 15, 17, 18, 18, 18, 19, 10, 15, 15, 15, 14, -5, 24, -7, -8, -9, -9, -11, 10, -6, 31, -15, 14, 17, 17, 17, 18, 19, 20, 19, 3, 14, 14, 14, 15, 16, 9, -5, -6, -6, -9, -9, -9, 29, 15, 16, 15, 2, -7, -9, -9, -10, 3, 8, 7, -3, -4, 0, 5, -4, -6, -9, 2, -6, -4, -3, 11, 12, 14, 15, 16, 18, 18, 18, 18, 18, 12, -4, 28, 0, 1, 2, -3, -9, 3, 12, 12, 15, 15, 17, 17, 16, 16, 15, 15, -6, 0, -5, -2, 0, 3, 12, 13, 14, -2, 0, 0, 0, -3, -8, -4, -11, -12, -13, -13, -21, -18, -11, -15, -8, -12, -14, -13, -11, -9, -9};

// scaler values
const double min_values[5] = { 0. ,  0. ,  0. , 0. , -36.};
const double scale_values[5] = {0.00461173, 0.00015954, 0.00376053, 0.00355971, 0.00609764};

// Input vector
double X[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
// Output vector
double output_mstedarls[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
double output_mptedarls[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

// Models
int n_features = 5;
MSTEDARLS mstedarls(
  8.414,    // threshold
  0.7,      // rls_mu
  1000.0,   // rls_delta
  1.0,      // w_init
  n_features,
  true      // correct_outlier
);
MPTEDARLS mptedarls(
    5.592,                       // threshold
    n_features,                 // rls_n
    0.9249,                     // rls_mu
    0.1,                        // rls_delta
    std::vector<double>(n_features, 0.0), // w_init
    true,                       // correct_outlier
    0,                          // window_size
    5,                          // window_outlier_limit
    false,                      // use_per_dim_teda
    6.0,                        // ecc_div
    1e-6,                       // epsilon
    true,                       // clip_output
    true,                       // clip_weights
    {-100.0, 100.0},            // output_clip_range
    {-100.0, 100.0},            // weight_clip_range
    10.0,                       // max_dw
    false                       // verbose
);


time_t timestamp;
long int time_session;
int prediction;
extern long int time_server;
extern bool time_server_obtained;
bool send_data = false;
int samples_sent = 0;
bool time_obtained = false;


void verifyAndResetFile(const char* filename) {
  if (SD.exists(filename)) {
    Serial.println(String(filename) + " exists. Creating a new file.");
    SD.remove(filename); // Remove o arquivo existente
  }

  File file = SD.open(filename, FILE_APPEND); // Cria um novo arquivo vazio
  if (!file) {
    Serial.println("Failed to create file: " + String(filename));
  } else {
    Serial.println("New file created: " + String(filename));

    // Escreve o cabeçalho
    file.print("sample");

    // Entradas originais
    file.print(",input_speed,input_rpm,input_tp,input_load,input_timing");

    // Saída MSTEDARLS
    file.print(",mstedarls_speed,mstedarls_rpm,mstedarls_tp,mstedarls_load,mstedarls_timing");
    file.print(",mstedarls_flag_speed,mstedarls_flag_rpm,mstedarls_flag_tp,mstedarls_flag_load,mstedarls_flag_timing");

    // Saída MPTEDARLS
    file.print(",mptedarls_speed,mptedarls_rpm,mptedarls_tp,mptedarls_load,mptedarls_timing");
    file.print(",mptedarls_flag");

    // Tempos de inferência
    file.print(",time_mstedarls_us,time_mptedarls_us");

    // Finaliza a linha do cabeçalho
    file.println();

    file.close();
  }
}

void obtainTimeTelelogger()
{
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    // sntp_setservername(0, (char*)"pool.ntp.org");
    sntp_setservername(0, (char*)"a.st1.ntp.br");
    sntp_init();

    time_obtained = true;
}

void verifyIfFileExists(String filename){
    if (!SD.exists(filename)) {
        Serial.println(filename + " does not exist");
        // Cria um novo arquivo com valor inicial (0 ou outro valor padrão)
        File file = SD.open(filename, FILE_WRITE);
        if (!file) {
            Serial.println("Failed to create file for writing");
            return;
        }
        file.println(0); // Escreve o valor inicial
        file.close();
    }
}

CBufferManager bufman;
Task subtask;

#if ENABLE_MEMS
float accBias[3] = {0}; // calibrated reference accelerometer data
float accSum[3] = {0};
float acc[3] = {0};
float gyr[3] = {0};
float mag[3] = {0};
uint8_t accCount = 0;
#endif
int deviceTemp = 0;

// config data
char apn[32];
#if ENABLE_WIFI
char wifiSSID[32] = WIFI_SSID;
char wifiPassword[32] = WIFI_PASSWORD;
#endif
nvs_handle_t nvs;

// live data
String netop;
String ip;
int16_t rssi = 0;
int16_t rssiLast = 0;
char vin[18] = {0};
uint16_t dtc[6] = {0};
float batteryVoltage = 0;
GPS_DATA* gd = 0;

char devid[12] = {0};
char isoTime[32] = {0};

// stats data
uint32_t lastMotionTime = 0;
uint32_t timeoutsOBD = 0;
uint32_t timeoutsNet = 0;
uint32_t lastStatsTime = 0;

int32_t syncInterval = SERVER_SYNC_INTERVAL * 1000;
int32_t dataInterval = 1000;

#if STORAGE != STORAGE_NONE
int fileid = 0;
uint16_t lastSizeKB = 0;
#endif

byte ledMode = 0;

bool serverSetup(IPAddress& ip);
void serverProcess(int timeout);
void processMEMS(CBuffer* buffer);
bool processGPS(CBuffer* buffer);
void processBLE(int timeout);

class State {
public:
  bool check(uint16_t flags) { return (m_state & flags) == flags; }
  void set(uint16_t flags) { m_state |= flags; }
  void clear(uint16_t flags) { m_state &= ~flags; }
  uint16_t m_state = 0;
};

FreematicsESP32 sys;

class OBD : public COBD
{
protected:
  void idleTasks()
  {
    // do some quick tasks while waiting for OBD response
#if ENABLE_MEMS
    processMEMS(0);
#endif
    processBLE(0);
  }
};

OBD obd;

MEMS_I2C* mems = 0;

#if STORAGE == STORAGE_SPIFFS
SPIFFSLogger logger;
#elif STORAGE == STORAGE_SD
SDLogger logger;
#endif

#if SERVER_PROTOCOL == PROTOCOL_UDP
TeleClientUDP teleClient;
#else
TeleClientHTTP teleClient;
#endif

#if ENABLE_OLED
OLED_SH1106 oled;
#endif

State state;

void printTimeoutStats()
{
  Serial.print("Timeouts: OBD:");
  Serial.print(timeoutsOBD);
  Serial.print(" Network:");
  Serial.println(timeoutsNet);
}

void beep(int duration)
{
    // turn on buzzer at 2000Hz frequency 
    sys.buzzer(2000);
    delay(duration);
    // turn off buzzer
    sys.buzzer(0);
}

#if LOG_EXT_SENSORS
void processExtInputs(CBuffer* buffer)
{
#if LOG_EXT_SENSORS == 1
  uint8_t levels[2] = {(uint8_t)digitalRead(PIN_SENSOR1), (uint8_t)digitalRead(PIN_SENSOR2)};
  buffer->add(PID_EXT_SENSORS, ELEMENT_UINT8, levels, sizeof(levels), 2);
#elif LOG_EXT_SENSORS == 2
  uint16_t reading[] = {adc1_get_raw(ADC1_CHANNEL_0), adc1_get_raw(ADC1_CHANNEL_1)};
  Serial.print("GPIO0:");
  Serial.print((float)reading[0] * 3.15 / 4095 - 0.01);
  Serial.print(" GPIO1:");
  Serial.println((float)reading[1] * 3.15 / 4095 - 0.01);
  buffer->add(PID_EXT_SENSORS, ELEMENT_UINT16, reading, sizeof(reading), 2);
#endif
}
#endif

/*******************************************************************************
  HTTP API
*******************************************************************************/
#if ENABLE_HTTPD
int handlerLiveData(UrlHandlerParam* param)
{
    char *buf = param->pucBuffer;
    int bufsize = param->bufSize;
    int n = snprintf(buf, bufsize, "{\"obd\":{\"vin\":\"%s\",\"battery\":%.1f,\"pid\":[", vin, batteryVoltage);
    uint32_t t = millis();
    for (int i = 0; i < sizeof(obdData) / sizeof(obdData[0]); i++) {
        n += snprintf(buf + n, bufsize - n, "{\"pid\":%u,\"value\":%d,\"age\":%u},",
            0x100 | obdData[i].pid, obdData[i].value, (unsigned int)(t - obdData[i].ts));
    }
    n--;
    n += snprintf(buf + n, bufsize - n, "]}");
#if ENABLE_MEMS
    if (accCount) {
      n += snprintf(buf + n, bufsize - n, ",\"mems\":{\"acc\":[%d,%d,%d],\"stationary\":%u}",
          (int)((accSum[0] / accCount - accBias[0]) * 100), (int)((accSum[1] / accCount - accBias[1]) * 100), (int)((accSum[2] / accCount - accBias[2]) * 100),
          (unsigned int)(millis() - lastMotionTime));
    }
#endif
    if (gd && gd->ts) {
      n += snprintf(buf + n, bufsize - n, ",\"gps\":{\"utc\":\"%s\",\"lat\":%f,\"lng\":%f,\"alt\":%f,\"speed\":%f,\"sat\":%d,\"age\":%u}",
          isoTime, gd->lat, gd->lng, gd->alt, gd->speed, (int)gd->sat, (unsigned int)(millis() - gd->ts));
    }
    buf[n++] = '}';
    param->contentLength = n;
    param->contentType=HTTPFILETYPE_JSON;
    return FLAG_DATA_RAW;
}
#endif

/*******************************************************************************
  Reading and processing OBD data
*******************************************************************************/
#if ENABLE_OBD

int readFlag = 0; 
uint8_t time_sd_flag;

void processOBD(CBuffer* buffer)
{

  static int idx[2] = {0, 0};
  int tier = 1;

  // while(time_session < 100000){
    
  //   // 1. Read the time from the RTC
  //   struct timeval tv;
  //   gettimeofday(&tv, NULL);
  //   timestamp = tv.tv_sec;
  //   time_session = timestamp;
  //   Serial.println("TIME SESSION");
  //   Serial.println(time_session);
  //   delay(1000);
  // }

  if (!time_server_obtained){
    time_session = 0;
    Serial.println("TIME SERVER NOT OBTAINED");
  } else {
    Serial.println("TIME SERVER OBTAINED");
    Serial.println(time_server);
    time_session = time_server;
  }

  if (count_date == 0 && time_server_obtained){
    // syncronize the timestamp with the server
    struct timeval tv;
    gettimeofday(&tv, NULL);
    tv.tv_sec = time_server;
    timestamp = tv.tv_sec;
  }

  // Serial.println("TIME STAMP");
  // Serial.println(timestamp);
  // file.print(timestamp);
  // file.close();

  if (!time_compared && time_server_obtained){

    // Verify if the file time.txt exists
    verifyIfFileExists("/time.txt");

    // Verify if the file time_session.txt exists
    verifyIfFileExists("/time_session.txt");

    // 2. Compare the time with the last time the data was sent. This time is stored in the "time.txt" file
    File file = SD.open("/time.txt", FILE_READ);
    if (!file) {
      Serial.println("Failed to open file for reading");
      return;
    }

    String lastTimeStr = file.readStringUntil('\n');
    file.close();

    // Serial.println("LAST TIME STRING");
    // Serial.println(lastTimeStr);

    // transform the string into a time_t
    time_t last_time = lastTimeStr.toInt();
    // Serial.println("LAST TIME");
    // Serial.println(last_time);

    // get the absolute difference between time_session and lastTime
    int diff = timestamp - last_time;
    Serial.println("DIFFERENCE");
    Serial.println(diff);

    // 3. If the difference is greater than 120 seconds, a new session has started
    if (diff > 120){
      // print the time_session
      Serial.println("A NEW SESSION HAS STARTED");
      
      time_session = timestamp;
      Serial.println(time_session);
      // update the time_session in the file
      File file = SD.open("/time_session.txt", FILE_WRITE);
      if (!file) {
        Serial.println("Failed to open file for writing");
        return;
      }

      // 5. In the first case, update the time_session in the file
      file.print(time_session);
      file.close();
    } else {
      // 4. If the difference is less than 120 seconds, the last session has not finished yet.
      Serial.println("THE LAST SESSION HAS NOT FINISHED YET");
      
      File file = SD.open("/time_session.txt", FILE_READ);
      if (!file) {
        Serial.println("Failed to open file for reading");
        return;
      }

      String lastTimeStr = file.readStringUntil('\n');
      file.close();

      // 6. In the second case, read the time_session from the file
      time_session = lastTimeStr.toInt();
    }

    time_compared = true;
  }

  // 7. Write the time_stamp in the file to register the last time the data was sent
  File file = SD.open("/time.txt", FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  struct timeval tv;
  gettimeofday(&tv, NULL);
  timestamp = tv.tv_sec;
  // Serial.println("TIME STAMP");
  // Serial.println(timestamp);
  file.print(timestamp);
  file.close();

  made_prediction = false;

  // -------------------------------------------------------------------------------------

  buffer->add(time_session_flag, ELEMENT_INT32, &time_session, sizeof(time_session));

  for (byte i = 0; i < sizeof(obdData) / sizeof(obdData[0]); i++) {
    if (obdData[i].tier > tier) {
        // reset previous tier index
        idx[tier - 2] = 0;
        // keep new tier number
        tier = obdData[i].tier;
        // move up current tier index
        i += idx[tier - 2]++;
        // check if into next tier
        if (obdData[i].tier != tier) {
            idx[tier - 2]= 0;
            i--;
            continue;
        }
    }
    byte pid = obdData[i].pid;
    if (!obd.isValidPID(pid)){
      // print the PID that is not valid
      Serial.print("PID ");
      Serial.print(pid, HEX);
      Serial.println(" is not valid");
      continue;
    }

    int value;
    bool success = false;
    if (obd.readPID(pid, value)) {
        obdData[i].ts = millis();
        obdData[i].value = value;

        // start
        // gettimeofday(&total_start, NULL);

        if (pid == PID_TIMING_ADVANCE) {
          count_model++;
          X[4] = value;
          buffer->add((uint16_t)pid | 0x100, ELEMENT_INT32, &value, sizeof(value));
        } else if (pid == PID_RPM) {
          count_model++;
          X[1] = value;
          buffer->add((uint16_t)pid | 0x100, ELEMENT_INT32, &value, sizeof(value));
        } else if (pid == PID_SPEED) {
          count_model++;
          X[0] = value;
          buffer->add((uint16_t)pid | 0x100, ELEMENT_INT32, &value, sizeof(value));
        } else if (pid == PID_THROTTLE) {
          count_model++;
          X[2] = value;
          buffer->add((uint16_t)pid | 0x100, ELEMENT_INT32, &value, sizeof(value));
        } else if (pid == PID_ENGINE_LOAD) {
          count_model++;
          X[3] = value;
          buffer->add((uint16_t)pid | 0x100, ELEMENT_INT32, &value, sizeof(value));
        } else {
          buffer->add((uint16_t)pid | 0x100, ELEMENT_INT32, &value, sizeof(value));
        }

        if (count_model == 5){

          std::vector<double> x_vec_mstedarls(X, X + n_features);

          // Execução do MSTEDARLS
          unsigned long start_time_mstedarls = micros();
          auto result_mstedarls = mstedarls.update(x_vec_mstedarls);
          unsigned long end_time_mstedarls = micros();
          unsigned long inference_time_mstedarls = end_time_mstedarls - start_time_mstedarls;

          std::vector<double> output_mstedarls = result_mstedarls.first;
          std::vector<bool> flags_mstedarls = result_mstedarls.second;

          // 2. Normaliza entrada para o MPTEDARLS
          double X_norm[5];
          for (int i = 0; i < n_features; ++i)
              X_norm[i] = (X[i] - min_values[i]) * scale_values[i];

          std::vector<double> x_vec(X_norm, X_norm + n_features);

          // Execução do MPTEDARLS com novo formato
          unsigned long start_time_mptedarls = micros();
          MPTEDARLS::RunResult result = mptedarls.run(x_vec);
          unsigned long end_time_mptedarls = micros();
          unsigned long inference_time_mptedarls = end_time_mptedarls - start_time_mptedarls;

          // 3. Desnormaliza a saída corrigida
          std::vector<double> output_mptedarls(n_features);
          for (int i = 0; i < n_features; ++i)
              output_mptedarls[i] = result.x_filtered[i] / scale_values[i] + min_values[i];

          // 4. Grava os dados no arquivo
          File logFile = SD.open("/data_mstedarls_mptedarls_polo.txt", FILE_APPEND);
          if (logFile) {
              logFile.print(samples_sent); logFile.print(",");

              // Entrada original
              for (int i = 0; i < n_features; ++i) {
                  logFile.print(X[i]); logFile.print(",");
              }

              // Saída do MSTEDARLS
              for (int i = 0; i < n_features; ++i) {
                  logFile.print(output_mstedarls[i]); logFile.print(",");
              }

              // Flags do MSTEDARLS
              for (int i = 0; i < n_features; ++i) {
                  logFile.print(flags_mstedarls[i] ? 1 : 0); logFile.print(",");
              }

              // Saída do MPTEDARLS
              for (int i = 0; i < n_features; ++i) {
                  logFile.print(output_mptedarls[i]); logFile.print(",");
              }

              // Flag global do MPTEDARLS
              logFile.print(result.outlier_flag ? 1 : 0); logFile.print(",");

              // Tempos de inferência
              logFile.print(inference_time_mstedarls); logFile.print(",");
              logFile.print(inference_time_mptedarls);
              logFile.println();

              logFile.close();
          } else {
              Serial.println("Erro ao abrir data_mstedarls_mptedarls.txt para escrita.");
          }

          count_model = 0;

        }
        
    } else {
        timeoutsOBD++;
        printTimeoutStats();
        break;
    }

    if (tier > 1) break;

    if (time_obtained){
      samples_sent++;
    }
    
  }

  int kph = obdData[0].value;
  if (kph >= 2) lastMotionTime = millis();
}
#endif

bool initGPS()
{
  // start GNSS receiver
  if (sys.gpsBeginExt()) {
    Serial.println("GNSS:OK(E)");
  } else if (sys.gpsBegin()) {
    Serial.println("GNSS:OK(I)");
  } else {
    Serial.println("GNSS:NO");
    return false;
  }
  return true;
}

bool processGPS(CBuffer* buffer)
{
  static uint32_t lastGPStime = 0;
  static uint32_t lastGPStick = 0;
  static float lastGPSLat = 0;
  static float lastGPSLng = 0;

  if (!gd) {
    lastGPStime = 0;
    lastGPSLat = 0;
    lastGPSLng = 0;
  }
#if GNSS == GNSS_STANDALONE
  if (state.check(STATE_GPS_READY)) {
    // read parsed GPS data
    if (!sys.gpsGetData(&gd)) {
      return false;
    }
  }
#else
    if (!teleClient.cell.getLocation(&gd)) {
      return false;
    }
#endif


  if (!gd || lastGPStime == gd->time || (gd->lng == 0 && gd->lat == 0)) {
#if GNSS_RESET_TIMEOUT
    if (millis() - lastGPStick > GNSS_RESET_TIMEOUT * 1000) {
      sys.gpsEnd();
      delay(50);
      initGPS();
      lastGPStick = millis();
    }
#endif
    return false;
  }
  lastGPStick = millis();

  if ((lastGPSLat || lastGPSLng) && (abs(gd->lat - lastGPSLat) > 0.001 || abs(gd->lng - lastGPSLng) > 0.001)) {
    // invalid coordinates data
    lastGPSLat = 0;
    lastGPSLng = 0;
    return false;
  }
  lastGPSLat = gd->lat;
  lastGPSLng = gd->lng;

  float kph = gd->speed * 1.852f;
  if (kph >= 2) lastMotionTime = millis();

  if (buffer) {
    buffer->add(PID_GPS_TIME, ELEMENT_UINT32, &gd->time, sizeof(uint32_t));
    buffer->add(PID_GPS_LATITUDE, ELEMENT_FLOAT, &gd->lat, sizeof(float));
    buffer->add(PID_GPS_LONGITUDE, ELEMENT_FLOAT, &gd->lng, sizeof(float));
    buffer->add(PID_GPS_ALTITUDE, ELEMENT_FLOAT_D1, &gd->alt, sizeof(float)); /* m */
    buffer->add(PID_GPS_SPEED, ELEMENT_FLOAT_D1, &kph, sizeof(kph));
    buffer->add(PID_GPS_HEADING, ELEMENT_UINT16, &gd->heading, sizeof(uint16_t));
    if (gd->sat) buffer->add(PID_GPS_SAT_COUNT, ELEMENT_UINT8, &gd->sat, sizeof(uint8_t));
    if (gd->hdop) buffer->add(PID_GPS_HDOP, ELEMENT_UINT8, &gd->hdop, sizeof(uint8_t));
  }
  
  // generate ISO time string
  // last_time_syncronized = gd->time;

  // if (last_time_syncronized > 1718389103 && !syncronized){
  //   syncronized = true;
  // }
  char *p = isoTime + sprintf(isoTime, "%04u-%02u-%02uT%02u:%02u:%02u",
      (unsigned int)(gd->date % 100) + 2000, (unsigned int)(gd->date / 100) % 100, (unsigned int)(gd->date / 10000),
      (unsigned int)(gd->time / 1000000), (unsigned int)(gd->time % 1000000) / 10000, (unsigned int)(gd->time % 10000) / 100);
  unsigned char tenth = (gd->time % 100) / 10;
  if (tenth) p += sprintf(p, ".%c00", '0' + tenth);
  *p = 'Z';
  *(p + 1) = 0;

  Serial.print("[GPS] ");
  Serial.print(gd->lat, 6);
  Serial.print(' ');
  Serial.print(gd->lng, 6);
  Serial.print(' ');
  Serial.print((int)kph);
  Serial.print("km/h");
  Serial.print(" SATS:");
  Serial.print(gd->sat);
  Serial.print(" HDOP:");
  Serial.print(gd->hdop);
  Serial.print(" Course:");
  Serial.print(gd->heading);

  Serial.print(' ');
  Serial.println(isoTime);
  //Serial.println(gd->errors);
  lastGPStime = gd->time;
  return true;
}

bool waitMotionGPS(int timeout)
{
  unsigned long t = millis();
  lastMotionTime = 0;
  do {
      serverProcess(100);
    if (!processGPS(0)) continue;
    if (lastMotionTime) return true;
  } while (millis() - t < timeout);
  return false;
}

#if ENABLE_MEMS
void processMEMS(CBuffer* buffer)
{
  if (!state.check(STATE_MEMS_READY)) return;

  // load and store accelerometer data
  float temp;
#if ENABLE_ORIENTATION
  ORIENTATION ori;
  if (!mems->read(acc, gyr, mag, &temp, &ori)) return;
#else
  if (!mems->read(acc, gyr, mag, &temp)) return;
#endif
  deviceTemp = (int)temp;

  accSum[0] += acc[0];
  accSum[1] += acc[1];
  accSum[2] += acc[2];
  accCount++;

  if (buffer) {
    if (accCount) {
      float value[3];
      value[0] = accSum[0] / accCount - accBias[0];
      value[1] = accSum[1] / accCount - accBias[1];
      value[2] = accSum[2] / accCount - accBias[2];
      buffer->add(PID_ACC, ELEMENT_FLOAT_D2, value, sizeof(value), 3);
/*
      Serial.print("[ACC] ");
      Serial.print(value[0]);
      Serial.print('/');
      Serial.print(value[1]);
      Serial.print('/');
      Serial.println(value[2]);
*/
#if ENABLE_ORIENTATION
      value[0] = ori.yaw;
      value[1] = ori.pitch;
      value[2] = ori.roll;
      buffer->add(PID_ORIENTATION, ELEMENT_FLOAT_D2, value, sizeof(value), 3);
#endif
#if 0
      // calculate motion
      float motion = 0;
      for (byte i = 0; i < 3; i++) {
        motion += value[i] * value[i];
      }
      if (motion >= MOTION_THRESHOLD * MOTION_THRESHOLD) {
        lastMotionTime = millis();
        Serial.print("Motion:");
        Serial.println(motion);
      }
#endif
    }
    accSum[0] = 0;
    accSum[1] = 0;
    accSum[2] = 0;
    accCount = 0;
  }
}

void calibrateMEMS()
{
  if (state.check(STATE_MEMS_READY)) {
    accBias[0] = 0;
    accBias[1] = 0;
    accBias[2] = 0;
    int n;
    unsigned long t = millis();
    for (n = 0; millis() - t < 1000; n++) {
      float acc[3];
      if (!mems->read(acc)) continue;
      accBias[0] += acc[0];
      accBias[1] += acc[1];
      accBias[2] += acc[2];
      delay(10);
    }
    accBias[0] /= n;
    accBias[1] /= n;
    accBias[2] /= n;
    Serial.print("ACC BIAS:");
    Serial.print(accBias[0]);
    Serial.print('/');
    Serial.print(accBias[1]);
    Serial.print('/');
    Serial.println(accBias[2]);
  }
}
#endif

void printTime()
{
  time_t utc;
  time(&utc);
  struct tm *btm = gmtime(&utc);
  if (btm->tm_year > 100) {
    // valid system time available
    char buf[64];
    sprintf(buf, "%04u-%02u-%02u %02u:%02u:%02u",
      1900 + btm->tm_year, btm->tm_mon + 1, btm->tm_mday, btm->tm_hour, btm->tm_min, btm->tm_sec);
    Serial.print("UTC:");
    Serial.println(buf);
  }
}

/*******************************************************************************
  Initializing all data logging components
*******************************************************************************/
void initialize()
{
  // dump buffer data
  bufman.purge();

#if ENABLE_MEMS
  if (state.check(STATE_MEMS_READY)) {
    calibrateMEMS();
  }
#endif

#if GNSS == GNSS_STANDALONE
  if (!state.check(STATE_GPS_READY)) {
    if (initGPS()) {
      state.set(STATE_GPS_READY);
    }
  }
#endif

#if ENABLE_OBD
  // initialize OBD communication
  if (!state.check(STATE_OBD_READY)) {
    timeoutsOBD = 0;
    if (obd.init()) {
      Serial.println("OBD:OK");
      state.set(STATE_OBD_READY);
#if ENABLE_OLED
      oled.println("OBD OK");
#endif
    } else {
      Serial.println("OBD:NO");
      //state.clear(STATE_WORKING);
      //return;
    }
  }
#endif

#if STORAGE != STORAGE_NONE
  if (!state.check(STATE_STORAGE_READY)) {
    // init storage
    if (logger.init()) {
      state.set(STATE_STORAGE_READY);
    }
  }
  if (state.check(STATE_STORAGE_READY)) {
    fileid = logger.begin();
  }
#endif

  // re-try OBD if connection not established
#if ENABLE_OBD
  if (state.check(STATE_OBD_READY)) {
    char buf[128];
    if (obd.getVIN(buf, sizeof(buf))) {
      memcpy(vin, buf, sizeof(vin) - 1);
      Serial.print("VIN:");
      Serial.println(vin);
    }
    int dtcCount = obd.readDTC(dtc, sizeof(dtc) / sizeof(dtc[0]));
    if (dtcCount > 0) {
      Serial.print("DTC:");
      Serial.println(dtcCount);
    }
#if ENABLE_OLED
    oled.print("VIN:");
    oled.println(vin);
#endif
  }
#endif

  // check system time
  printTime();

  lastMotionTime = millis();
  state.set(STATE_WORKING);

#if ENABLE_OLED
  delay(1000);
  oled.clear();
  oled.print("DEVICE ID: ");
  oled.println(devid);
  oled.setCursor(0, 7);
  oled.print("Packets");
  oled.setCursor(80, 7);
  oled.print("KB Sent");
  oled.setFontSize(FONT_SIZE_MEDIUM);
#endif
}

void showStats()
{
  uint32_t t = millis() - teleClient.startTime;
  char buf[32];
  sprintf(buf, "%02u:%02u.%c ", t / 60000, (t % 60000) / 1000, (t % 1000) / 100 + '0');
  Serial.print("[NET] ");
  Serial.print(buf);
  Serial.print("| Packet #");
  Serial.print(teleClient.txCount);
  Serial.print(" | Out: ");
  Serial.print(teleClient.txBytes >> 10);
  Serial.print(" KB | In: ");
  Serial.print(teleClient.rxBytes);
  Serial.print(" bytes | ");
  Serial.print((unsigned int)((uint64_t)(teleClient.txBytes + teleClient.rxBytes) * 3600 / (millis() - teleClient.startTime)));
  Serial.print(" KB/h");

  Serial.println();
#if ENABLE_OLED
  oled.setCursor(0, 2);
  oled.println(timestr);
  oled.setCursor(0, 5);
  oled.printInt(teleClient.txCount, 2);
  oled.setCursor(80, 5);
  oled.printInt(teleClient.txBytes >> 10, 3);
#endif
}

bool waitMotion(long timeout)
{
#if ENABLE_MEMS
  unsigned long t = millis();
  if (state.check(STATE_MEMS_READY)) {
    do {
      // calculate relative movement
      float motion = 0;
      float acc[3];
      if (!mems->read(acc)) continue;
      if (accCount == 10) {
        accCount = 0;
        accSum[0] = 0;
        accSum[1] = 0;
        accSum[2] = 0;
      }
      accSum[0] += acc[0];
      accSum[1] += acc[1];
      accSum[2] += acc[2];
      accCount++;
      for (byte i = 0; i < 3; i++) {
        float m = (acc[i] - accBias[i]);
        motion += m * m;
      }
#if ENABLE_HTTTPD
      serverProcess(100);
#endif
      processBLE(100);
      // check movement
      if (motion >= MOTION_THRESHOLD * MOTION_THRESHOLD) {
        //lastMotionTime = millis();
        Serial.println(motion);
        return true;
      }
    } while (state.check(STATE_STANDBY) && ((long)(millis() - t) < timeout || timeout == -1));
    return false;
  }
#endif
  serverProcess(timeout);
  return false;
}

/*******************************************************************************
  Collecting and processing data
*******************************************************************************/
void process()
{
  uint32_t startTime = millis();

  CBuffer* buffer = bufman.getFree();
  buffer->state = BUFFER_STATE_FILLING;

#if ENABLE_OBD
  // process OBD data if connected
  if (state.check(STATE_OBD_READY)) {
      processOBD(buffer);
    if (obd.errors >= MAX_OBD_ERRORS) {
      if (!obd.init()) {
        Serial.println("[OBD] ECU OFF");
        state.clear(STATE_OBD_READY | STATE_WORKING);
        return;
      }
    }
  } else if (obd.init(PROTO_AUTO, true)) {
    state.set(STATE_OBD_READY);
    Serial.println("[OBD] ECU ON");
  }
#endif

  if (rssi != rssiLast) {
    int val = (rssiLast = rssi);
    buffer->add(PID_CSQ, ELEMENT_INT32, &val, sizeof(val));
  }
#if ENABLE_OBD
  if (sys.devType > 12) {
    batteryVoltage = (float)(analogRead(A0) * 45) / 4095;
  } else {
    batteryVoltage = obd.getVoltage();
  }
  if (batteryVoltage) {
    uint16_t v = batteryVoltage * 100;
    buffer->add(PID_BATTERY_VOLTAGE, ELEMENT_UINT16, &v, sizeof(v));
  }
#endif

#if LOG_EXT_SENSORS
  processExtInputs(buffer);
#endif

#if ENABLE_MEMS
  processMEMS(buffer);
#endif

  processGPS(buffer);

  if (!state.check(STATE_MEMS_READY)) {
    deviceTemp = readChipTemperature();
  }
  buffer->add(PID_DEVICE_TEMP, ELEMENT_INT32, &deviceTemp, sizeof(deviceTemp));

  buffer->timestamp = millis();
  
  buffer->state = BUFFER_STATE_FILLED;

  // display file buffer stats
  if (startTime - lastStatsTime >= 3000) {
    bufman.printStats();
    lastStatsTime = startTime;
  }

#if STORAGE != STORAGE_NONE
  if (state.check(STATE_STORAGE_READY)) {
    buffer->serialize(logger);
    uint16_t sizeKB = (uint16_t)(logger.size() >> 10);
    if (sizeKB != lastSizeKB) {
      logger.flush();
      lastSizeKB = sizeKB;
      Serial.print("[FILE] ");
      Serial.print(sizeKB);
      Serial.println("KB");
    }
  }
#endif

  const int dataIntervals[] = DATA_INTERVAL_TABLE;
#if ENABLE_OBD || ENABLE_MEMS
  // motion adaptive data interval control
  const uint16_t stationaryTime[] = STATIONARY_TIME_TABLE;
  unsigned int motionless = (millis() - lastMotionTime) / 1000;
  bool stationary = true;
  for (byte i = 0; i < sizeof(stationaryTime) / sizeof(stationaryTime[0]); i++) {
    dataInterval = dataIntervals[i];
    if (motionless < stationaryTime[i] || stationaryTime[i] == 0) {
      stationary = false;
      break;
    }
  }
  if (stationary) {
    // stationery timeout
    Serial.print("Stationary for ");
    Serial.print(motionless);
    Serial.println(" secs");
    // trip ended, go into standby
    state.clear(STATE_WORKING);
    return;
  }
#else
  dataInterval = dataIntervals[0];
#endif
  do {
    long t = dataInterval - (millis() - startTime);
    processBLE(t > 0 ? t : 0);
  } while (millis() - startTime < dataInterval);
}

bool initCell(bool quick = false)
{
  Serial.println("[CELL] Activating...");
  // power on network module
  if (!teleClient.cell.begin(&sys)) {
    Serial.println("[CELL] No supported module");
#if ENABLE_OLED
    oled.println("No Cell Module");
#endif
    return false;
  }
  if (quick) return true;
#if ENABLE_OLED
    oled.print(teleClient.cell.deviceName());
    oled.println(" OK\r");
    oled.print("IMEI:");
    oled.println(teleClient.cell.IMEI);
#endif
  Serial.print("CELL:");
  Serial.println(teleClient.cell.deviceName());
  if (!teleClient.cell.checkSIM(SIM_CARD_PIN)) {
    Serial.println("NO SIM CARD");
    //return false;
  }
  Serial.print("IMEI:");
  Serial.println(teleClient.cell.IMEI);
  Serial.println("[CELL] Searching...");
  if (*apn) {
    Serial.print("APN:");
    Serial.println(apn);
  }
  if (teleClient.cell.setup(apn, "claro", "claro")) {
    netop = teleClient.cell.getOperatorName();
    if (netop.length()) {
      Serial.print("Operator:");
      Serial.println(netop);
#if ENABLE_OLED
      oled.println(op);
#endif
    }

#if GNSS == GNSS_CELLULAR
    if (teleClient.cell.setGPS(true)) {
      Serial.println("CELL GNSS:OK");
    }
#endif

    ip = teleClient.cell.getIP();
    if (ip.length()) {
      Serial.print("[CELL] IP:");
      Serial.println(ip);
      // obtainTimeTelelogger();
#if ENABLE_OLED
      oled.print("IP:");
      oled.println(ip);
#endif
    }
    state.set(STATE_CELL_CONNECTED);
  } else {
    char *p = strstr(teleClient.cell.getBuffer(), "+CPSI:");
    if (p) {
      char *q = strchr(p, '\r');
      if (q) *q = 0;
      Serial.print("[CELL] ");
      Serial.println(p + 7);
#if ENABLE_OLED
      oled.println(p + 7);
#endif
    } else {
      Serial.print(teleClient.cell.getBuffer());
    }
  }
  timeoutsNet = 0;
  return state.check(STATE_CELL_CONNECTED);
}

/*******************************************************************************
  Initializing network, maintaining connection and doing transmissions
*******************************************************************************/
void telemetry(void* inst)
{
  uint32_t lastRssiTime = 0;
  uint8_t connErrors = 0;
  CStorageRAM store;
  store.init(
#if BOARD_HAS_PSRAM
    (char*)heap_caps_malloc(SERIALIZE_BUFFER_SIZE, MALLOC_CAP_SPIRAM),
#else
    (char*)malloc(SERIALIZE_BUFFER_SIZE),
#endif
    SERIALIZE_BUFFER_SIZE
  );
  teleClient.reset();

  for (;;) {
    if (state.check(STATE_STANDBY)) {
      if (state.check(STATE_CELL_CONNECTED) || state.check(STATE_WIFI_CONNECTED)) {
        teleClient.shutdown();
        netop = "";
        ip = "";
        rssi = 0;
      }
      state.clear(STATE_NET_READY | STATE_CELL_CONNECTED | STATE_WIFI_CONNECTED);
      teleClient.reset();
      bufman.purge();

      uint32_t t = millis();
      do {
        delay(1000);
      } while (state.check(STATE_STANDBY) && millis() - t < 1000L * PING_BACK_INTERVAL);
      if (state.check(STATE_STANDBY)) {
        // start ping
#if ENABLE_WIFI
        if (wifiSSID[0]) { 
          Serial.print("[WIFI] Joining SSID:");
          Serial.println(wifiSSID);
          teleClient.wifi.begin(wifiSSID, wifiPassword);
        }
        if (teleClient.wifi.setup()) {
          Serial.println("[WIFI] Ping...");
          teleClient.ping();
        }
        else
#endif
        {
          if (initCell()) {
            Serial.println("[CELL] Ping...");
            teleClient.ping();
          }
        }
        teleClient.shutdown();
        state.clear(STATE_CELL_CONNECTED | STATE_WIFI_CONNECTED);
      }
      continue;
    }

#if ENABLE_WIFI
    if (wifiSSID[0] && !state.check(STATE_WIFI_CONNECTED)) {
      Serial.print("[WIFI] Joining SSID:");
      Serial.println(wifiSSID);
      teleClient.wifi.begin(wifiSSID, wifiPassword);
      teleClient.wifi.setup();
    }
#endif

    while (state.check(STATE_WORKING)) {
#if ENABLE_WIFI
      if (wifiSSID[0]) {
        if (!state.check(STATE_WIFI_CONNECTED) && teleClient.wifi.connected()) {
          ip = teleClient.wifi.getIP();
          if (ip.length()) {
            Serial.print("[WIFI] IP:");
            Serial.println(ip);
          }

          if(!time_obtained){
            // obtainTimeTelelogger();
            // time_obtained = true;
          }
          connErrors = 0;
          if (teleClient.connect()) {
            state.set(STATE_WIFI_CONNECTED | STATE_NET_READY);
            beep(50);
            // switch off cellular module when wifi connected
            if (state.check(STATE_CELL_CONNECTED)) {
              teleClient.cell.end();
              state.clear(STATE_CELL_CONNECTED);
              Serial.println("[CELL] Deactivated");
            }
          }
        } else if (state.check(STATE_WIFI_CONNECTED) && !teleClient.wifi.connected()) {
          Serial.println("[WIFI] Disconnected");
          state.clear(STATE_WIFI_CONNECTED);
        }
      }
#endif
      if (!state.check(STATE_WIFI_CONNECTED) && !state.check(STATE_CELL_CONNECTED)) {
        connErrors = 0;
        if (!initCell() || !teleClient.connect()) {
          teleClient.cell.end();
          state.clear(STATE_NET_READY | STATE_CELL_CONNECTED);
          break;
        }
        Serial.println("[CELL] In service");
        state.set(STATE_NET_READY);
        beep(50);
      }

      if (millis() - lastRssiTime > SIGNAL_CHECK_INTERVAL * 1000) {
#if ENABLE_WIFI
        if (state.check(STATE_WIFI_CONNECTED))
        {
          rssi = teleClient.wifi.RSSI();
        }
        else
#endif
        {
          rssi = teleClient.cell.RSSI();
        }
        if (rssi) {
          Serial.print("RSSI:");
          Serial.print(rssi);
          Serial.println("dBm");
        }
        lastRssiTime = millis();

#if ENABLE_WIFI
        if (wifiSSID[0] && !state.check(STATE_WIFI_CONNECTED)) {
          teleClient.wifi.begin(wifiSSID, wifiPassword);
        }
#endif
      }

      // get data from buffer
      CBuffer* buffer = bufman.getNewest();
      if (!buffer) {
        delay(50);
        continue;
      }
#if SERVER_PROTOCOL == PROTOCOL_UDP
      store.header(devid);
#endif
      store.timestamp(buffer->timestamp);
      buffer->serialize(store);
      bufman.free(buffer);
      store.tailer();
      Serial.print("[DAT] ");
      Serial.println(store.buffer());

      // start transmission
#ifdef PIN_LED
      if (ledMode == 0) digitalWrite(PIN_LED, HIGH);
#endif

      if (teleClient.transmit(store.buffer(), store.length())) {
        // successfully sent
        connErrors = 0;
        showStats();
      } else {
        timeoutsNet++;
        connErrors++;
        printTimeoutStats();
        if (connErrors < MAX_CONN_ERRORS_RECONNECT) {
          // quick reconnect
          teleClient.connect(true);
        }
      }
#ifdef PIN_LED
      if (ledMode == 0) digitalWrite(PIN_LED, LOW);
#endif
      store.purge();

      teleClient.inbound();

      if (state.check(STATE_CELL_CONNECTED) && !teleClient.cell.check(1000)) {
        Serial.println("[CELL] Not in service");
        state.clear(STATE_NET_READY | STATE_CELL_CONNECTED);
        break;
      }

      if (syncInterval > 10000 && millis() - teleClient.lastSyncTime > syncInterval) {
        Serial.println("[NET] Poor connection");
        timeoutsNet++;
        if (!teleClient.connect()) {
          connErrors++;
        }
      }

      if (connErrors >= MAX_CONN_ERRORS_RECONNECT) {
#if ENABLE_WIFI
        if (state.check(STATE_WIFI_CONNECTED)) {
          teleClient.wifi.end();
          state.clear(STATE_NET_READY | STATE_WIFI_CONNECTED);
          break;
        }
#endif
        if (state.check(STATE_CELL_CONNECTED)) {
          teleClient.cell.end();
          state.clear(STATE_NET_READY | STATE_CELL_CONNECTED);
          break;
        }
      }

      if (deviceTemp >= COOLING_DOWN_TEMP) {
        // device too hot, cool down by pause transmission
        Serial.print("HIGH DEVICE TEMP: ");
        Serial.println(deviceTemp);
        bufman.purge();
      }

    }
  }
}

/*******************************************************************************
  Implementing stand-by mode
*******************************************************************************/
void standby()
{
  state.set(STATE_STANDBY);
#if STORAGE != STORAGE_NONE
  if (state.check(STATE_STORAGE_READY)) {
    logger.end();
  }
#endif

#if !GNSS_ALWAYS_ON && GNSS == GNSS_STANDALONE
  if (state.check(STATE_GPS_READY)) {
    Serial.println("[GPS] OFF");
    sys.gpsEnd(true);
    state.clear(STATE_GPS_READY);
    gd = 0;
  }
#endif

  state.clear(STATE_WORKING | STATE_OBD_READY | STATE_STORAGE_READY);
  // this will put co-processor into sleep mode
#if ENABLE_OLED
  oled.print("STANDBY");
  delay(1000);
  oled.clear();
#endif
  Serial.println("STANDBY");
  obd.enterLowPowerMode();
#if ENABLE_MEMS
  calibrateMEMS();
  waitMotion(-1);
#elif ENABLE_OBD
  do {
    delay(5000);
  } while (obd.getVoltage() < JUMPSTART_VOLTAGE);
#else
  delay(5000);
#endif
  Serial.println("WAKEUP");
  sys.resetLink();
#if RESET_AFTER_WAKEUP
#if ENABLE_MEMS
  if (mems) mems->end();  
#endif
  ESP.restart();
#endif  
  state.clear(STATE_STANDBY);
}

/*******************************************************************************
  Tasks to perform in idle/waiting time
*******************************************************************************/
void genDeviceID(char* buf)
{
    uint64_t seed = ESP.getEfuseMac() >> 8;
    for (int i = 0; i < 8; i++, seed >>= 5) {
      byte x = (byte)seed & 0x1f;
      if (x >= 10) {
        x = x - 10 + 'A';
        switch (x) {
          case 'B': x = 'W'; break;
          case 'D': x = 'X'; break;
          case 'I': x = 'Y'; break;
          case 'O': x = 'Z'; break;
        }
      } else {
        x += '0';
      }
      buf[i] = x;
    }
    buf[8] = 0;
}

void showSysInfo()
{
  Serial.print("CPU:");
  Serial.print(ESP.getCpuFreqMHz());
  Serial.print("MHz FLASH:");
  Serial.print(ESP.getFlashChipSize() >> 20);
  Serial.println("MB");
  Serial.print("IRAM:");
  Serial.print(ESP.getHeapSize() >> 10);
  Serial.print("KB");
#if BOARD_HAS_PSRAM
  if (psramInit()) {
    Serial.print(" PSRAM:");
    Serial.print(esp_spiram_get_size() >> 20);
    Serial.print("MB");
  }
#endif
  Serial.println();

  int rtc = rtc_clk_slow_freq_get();
  if (rtc) {
    Serial.print("RTC:");
    Serial.println(rtc);
  }

#if ENABLE_OLED
  oled.clear();
  oled.print("CPU:");
  oled.print(ESP.getCpuFreqMHz());
  oled.print("Mhz ");
  oled.print(getFlashSize() >> 10);
  oled.println("MB Flash");
#endif

  Serial.print("DEVICE ID:");
  Serial.println(devid);
#if ENABLE_OLED
  oled.print("DEVICE ID:");
  oled.println(devid);
#endif
}

void loadConfig()
{
  size_t len;
  len = sizeof(apn);
  apn[0] = 0;
  nvs_get_str(nvs, "CELL_APN", apn, &len);
  if (!apn[0]) {
    strcpy(apn, CELL_APN);
  }

#if ENABLE_WIFI
  len = sizeof(wifiSSID);
  nvs_get_str(nvs, "WIFI_SSID", wifiSSID, &len);
  len = sizeof(wifiPassword);
  nvs_get_str(nvs, "WIFI_PWD", wifiPassword, &len);
#endif
}

void processBLE(int timeout)
{
#if ENABLE_BLE
  static byte echo = 0;
  char* cmd;
  if (!(cmd = ble_recv_command(timeout))) {
    return;
  }

  char *p = strchr(cmd, '\r');
  if (p) *p = 0;
  char buf[48];
  int bufsize = sizeof(buf);
  int n = 0;
  if (echo) n += snprintf(buf + n, bufsize - n, "%s\r", cmd);
  Serial.print("[BLE] ");
  Serial.print(cmd);
  if (!strcmp(cmd, "UPTIME") || !strcmp(cmd, "TICK")) {
    n += snprintf(buf + n, bufsize - n, "%lu", millis());
  } else if (!strcmp(cmd, "BATT")) {
    n += snprintf(buf + n, bufsize - n, "%.2f", (float)(analogRead(A0) * 42) / 4095);
  } else if (!strcmp(cmd, "RESET")) {
#if STORAGE
    logger.end();
#endif
    ESP.restart();
    // never reach here
  } else if (!strcmp(cmd, "OFF")) {
    state.set(STATE_STANDBY);
    state.clear(STATE_WORKING);
    n += snprintf(buf + n, bufsize - n, "OK");
  } else if (!strcmp(cmd, "ON")) {
    state.clear(STATE_STANDBY);
    n += snprintf(buf + n, bufsize - n, "OK");
  } else if (!strcmp(cmd, "ON?")) {
    n += snprintf(buf + n, bufsize - n, "%u", state.check(STATE_STANDBY) ? 0 : 1);
  } else if (!strcmp(cmd, "APN?")) {
    n += snprintf(buf + n, bufsize - n, "%s", *apn ? apn : "DEFAULT");
  } else if (!strncmp(cmd, "APN=", 4)) {
    n += snprintf(buf + n, bufsize - n, nvs_set_str(nvs, "CELL_APN", strcmp(cmd + 4, "DEFAULT") ? cmd + 4 : "") == ESP_OK ? "OK" : "ERR");
    loadConfig();
  } else if (!strcmp(cmd, "NET_OP")) {
    if (state.check(STATE_WIFI_CONNECTED)) {
#if ENABLE_WIFI
      n += snprintf(buf + n, bufsize - n, "%s", wifiSSID[0] ? wifiSSID : "-");
#endif
    } else {
      snprintf(buf + n, bufsize - n, "%s", netop.length() ? netop.c_str() : "-");
      char *p = strchr(buf + n, ' ');
      if (p) *p = 0;
      n += strlen(buf + n);
    }
  } else if (!strcmp(cmd, "NET_IP")) {
    n += snprintf(buf + n, bufsize - n, "%s", ip.length() ? ip.c_str() : "-");
  } else if (!strcmp(cmd, "NET_PACKET")) {
      n += snprintf(buf + n, bufsize - n, "%u", teleClient.txCount);
  } else if (!strcmp(cmd, "NET_DATA")) {
      n += snprintf(buf + n, bufsize - n, "%u", teleClient.txBytes);
  } else if (!strcmp(cmd, "NET_RATE")) {
      n += snprintf(buf + n, bufsize - n, "%u", teleClient.startTime ? (unsigned int)((uint64_t)(teleClient.txBytes + teleClient.rxBytes) * 3600 / (millis() - teleClient.startTime)) : 0);
  } else if (!strcmp(cmd, "RSSI")) {
    n += snprintf(buf + n, bufsize - n, "%d", rssi);
#if ENABLE_WIFI
  } else if (!strcmp(cmd, "SSID?")) {
    n += snprintf(buf + n, bufsize - n, "%s", wifiSSID[0] ? wifiSSID : "-");
  } else if (!strncmp(cmd, "SSID=", 5)) {
    const char* p = cmd + 5;
    n += snprintf(buf + n, bufsize - n, nvs_set_str(nvs, "WIFI_SSID", strcmp(p, "-") ? p : "") == ESP_OK ? "OK" : "ERR");
    loadConfig();
  } else if (!strcmp(cmd, "WPWD?")) {
    n += snprintf(buf + n, bufsize - n, "%s", wifiPassword[0] ? wifiPassword : "-");
  } else if (!strncmp(cmd, "WPWD=", 5)) {
    const char* p = cmd + 5;
    n += snprintf(buf + n, bufsize - n, nvs_set_str(nvs, "WIFI_PWD", strcmp(p, "-") ? p : "") == ESP_OK ? "OK" : "ERR");
    loadConfig();
#else
  } else if (!strcmp(cmd, "SSID?") || !strcmp(cmd, "WPWD?")) {
    n += snprintf(buf + n, bufsize - n, "-");
#endif
#if ENABLE_MEMS
  } else if (!strcmp(cmd, "TEMP")) {
    n += snprintf(buf + n, bufsize - n, "%d", (int)deviceTemp);
  } else if (!strcmp(cmd, "ACC")) {
    n += snprintf(buf + n, bufsize - n, "%.1f/%.1f/%.1f", acc[0], acc[1], acc[2]);
  } else if (!strcmp(cmd, "GYRO")) {
    n += snprintf(buf + n, bufsize - n, "%.1f/%.1f/%.1f", gyr[0], gyr[1], gyr[2]);
  } else if (!strcmp(cmd, "GF")) {
    n += snprintf(buf + n, bufsize - n, "%f", (float)sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]));
#endif
  } else if (!strcmp(cmd, "ATE0")) {
    echo = 0;
    n += snprintf(buf + n, bufsize - n, "OK");
  } else if (!strcmp(cmd, "ATE1")) {
    echo = 1;
    n += snprintf(buf + n, bufsize - n, "OK");
  } else if (!strcmp(cmd, "FS")) {
    n += snprintf(buf + n, bufsize - n, "%u",
#if STORAGE == STORAGE_NONE
    0
#else
    logger.size()
#endif
      );
  } else if (!memcmp(cmd, "01", 2)) {
    byte pid = hex2uint8(cmd + 2);
    for (byte i = 0; i < sizeof(obdData) / sizeof(obdData[0]); i++) {
      if (obdData[i].pid == pid) {
        n += snprintf(buf + n, bufsize - n, "%d", obdData[i].value);
        pid = 0;
        break;
      }
    }
    if (pid) {
      int value;
      if (obd.readPID(pid, value)) {
        n += snprintf(buf + n, bufsize - n, "%d", value);
      } else {
        n += snprintf(buf + n, bufsize - n, "N/A");
      }
    }
  } else if (!strcmp(cmd, "VIN")) {
    n += snprintf(buf + n, bufsize - n, "%s", vin[0] ? vin : "N/A");
  } else if (!strcmp(cmd, "LAT") && gd) {
    n += snprintf(buf + n, bufsize - n, "%f", gd->lat);
  } else if (!strcmp(cmd, "LNG") && gd) {
    n += snprintf(buf + n, bufsize - n, "%f", gd->lng);
  } else if (!strcmp(cmd, "ALT") && gd) {
    n += snprintf(buf + n, bufsize - n, "%d", (int)gd->alt);
  } else if (!strcmp(cmd, "SAT") && gd) {
    n += snprintf(buf + n, bufsize - n, "%u", (unsigned int)gd->sat);
  } else if (!strcmp(cmd, "SPD") && gd) {
    n += snprintf(buf + n, bufsize - n, "%d", (int)(gd->speed * 1852 / 1000));
  } else if (!strcmp(cmd, "CRS") && gd) {
    n += snprintf(buf + n, bufsize - n, "%u", (unsigned int)gd->heading);
  } else {
    n += snprintf(buf + n, bufsize - n, "ERROR");
  }
  Serial.print(" -> ");
  Serial.println((p = strchr(buf, '\r')) ? p + 1 : buf);
  if (n < bufsize - 1) {
    buf[n++] = '\r';
  } else {
    n = bufsize - 1;
  }
  buf[n] = 0;
  ble_send_response(buf, n, cmd);
#else
  if (timeout) delay(timeout);
#endif
}

void setup()
{
  delay(500);

  // Initialize NVS
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // NVS partition was truncated and needs to be erased
    // Retry nvs_flash_init
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK( err );
  err = nvs_open("storage", NVS_READWRITE, &nvs);
  if (err == ESP_OK) {
    loadConfig();
  }

#if ENABLE_OLED
  oled.begin();
  oled.setFontSize(FONT_SIZE_SMALL);
#endif
  // initialize USB serial
  Serial.begin(115200);

  // init LED pin
#ifdef PIN_LED
  pinMode(PIN_LED, OUTPUT);
  if (ledMode == 0) digitalWrite(PIN_LED, HIGH);
#endif

  // generate unique device ID
  genDeviceID(devid);

#if CONFIG_MODE_TIMEOUT
  configMode();
#endif

#if LOG_EXT_SENSORS == 1
  pinMode(PIN_SENSOR1, INPUT);
  pinMode(PIN_SENSOR2, INPUT);
#elif LOG_EXT_SENSORS == 2
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_11);
#endif

  // show system information
  showSysInfo();

  bufman.init();
  
  //Serial.print(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) >> 10);
  //Serial.println("KB");

#if ENABLE_OBD
  if (sys.begin()) {
    Serial.print("TYPE:");
    Serial.println(sys.devType);
    obd.begin(sys.link);
  }
#else
  sys.begin(false, true);
#endif

#if ENABLE_MEMS
if (!state.check(STATE_MEMS_READY)) do {
  Serial.print("MEMS:");
  mems = new ICM_42627;
  byte ret = mems->begin();
  if (ret) {
    state.set(STATE_MEMS_READY);
    Serial.println("ICM-42627");
    break;
  }
  delete mems;
  mems = new ICM_20948_I2C;
  ret = mems->begin();
  if (ret) {
    state.set(STATE_MEMS_READY);
    Serial.println("ICM-20948");
    break;
  } 
  delete mems;
  /*
  mems = new MPU9250;
  ret = mems->begin();
  if (ret) {
    state.set(STATE_MEMS_READY);
    Serial.println("MPU-9250");
    break;
  }
  */
  mems = 0;
  Serial.println("NO");
} while (0);
#endif

#if ENABLE_HTTPD
  IPAddress ip;
  if (serverSetup(ip)) {
    Serial.println("HTTPD:");
    Serial.println(ip);
#if ENABLE_OLED
    oled.println(ip);
#endif
  } else {
    Serial.println("HTTPD:NO");
  }
#endif

  state.set(STATE_WORKING);

#if ENABLE_BLE
  // init BLE
  ble_init("FreematicsPlus");
#endif

  // initialize components
  initialize();

  // initialize network and maintain connection
  subtask.create(telemetry, "telemetry", 2, 8192);

#ifdef PIN_LED
  digitalWrite(PIN_LED, LOW);
#endif
}

void loop()
{
  // error handling
  if (!state.check(STATE_WORKING)) {
    standby();
#ifdef PIN_LED
    if (ledMode == 0) digitalWrite(PIN_LED, HIGH);
#endif
    initialize();
    verifyAndResetFile("/data_mstedarls_mptedarls_polo.txt");
#ifdef PIN_LED
    digitalWrite(PIN_LED, LOW);
#endif
    return;
  }

  // collect and log data
  process();
}