#include <vector>
using namespace std;

std::vector<std::vector<float>>                 *m_cameraMatrix;
std::vector<std::vector<std::vector<float>>>    m_vertices;

vector<vector<int>> COLOR_MAP       = {{0,113,188},{216,82,24},{236,176,31},
                                       {125,46,141},{118,171,47},{76,189,237},
                                       {161,19,46},{76,76,76},{153,153,153},
                                       {255,0,0},{255,127,0},{190,190,0},
                                       {0,255,0},{0,0,255},{170,0,255},
                                       {84,84,0},{84,170,0},{84,255,0},
                                       {170,84,0},{170,170,0},{170,255,0},
                                       {255,84,0},{255,170,0},{255,255,0},
                                       {0,84,127},{0,170,127},{0,255,127},
                                       {84,0,127},{84,84,127},{84,170,127},
                                       {84,255,127},{170,0,127},{170,84,127},
                                       {170,170,127},{170,255,127},{255,0,127},
                                       {255,84,127},{255,170,127},{255,255,127},
                                       {0,84,255},{0,170,255},{0,255,255},
                                       {84,0,255},{84,84,255},{84,170,255},
                                       {84,255,255},{170,0,255},{170,84,255},
                                       {170,170,255},{170,255,255},{255,0,255},
                                       {255,84,255},{255,170,255},{84,0,0},
                                       {127,0,0},{170,0,0},{212,0,0},
                                       {255,0,0},{0,42,0},{0,84,0},
                                       {0,127,0},{0,170,0},{0,212,0},
                                       {0,255,0},{0,0,42},{0,0,84},
                                       {0,0,127},{0,0,170},{0,0,212},
                                       {0,0,255},{0,0,0},{36,36,36},
                                       {72,72,72},{109,109,109},{145,145,145},
                                       {182,182,182},{218,218,218},{0,113,188},
                                       {80,182,188},{127,127,0}};

vector<vector<float>> YCBV_CAMERA_MATRIX = {{1066.778, 0, 312.9869},
                                           {0.0, 1067.487, 241.3109},
                                           {0.0, 0.0, 1.0}};

vector<vector<float>> LM_CAMERA_MATRIX  = {{572.4114, 0.0, 325.2611},
                                           {0.0, 573.57043, 242.04899},
                                           {0.0, 0.0, 1.0}};

vector<vector<float>> YCBV_VERTICES      = {{51.1445, 51.223, 70.072},
                                           {35.865, 81.9885, 106.743},
                                           {24.772, 47.024, 88.0075},
                                           {33.927, 33.875, 51.0185},
                                           {48.575, 33.31, 95.704},
                                           {42.755, 42.807, 16.7555},
                                           {68.924, 64.3955, 19.414},
                                           {44.6775, 50.5545, 15.06},
                                           {51.0615, 30.161, 41.8185},
                                           {54.444, 89.206, 18.335},
                                           {74.4985, 72.3845, 121.32},
                                           {51.203, 33.856, 125.32},
                                           {80.722, 80.5565, 27.485},
                                           {58.483, 46.5375, 40.692},
                                           {92.1205, 93.717, 28.6585},
                                           {51.9755, 51.774, 102.945},
                                           {48.04, 100.772, 7.858},
                                           {10.5195, 60.4225, 9.4385},
                                           {59.978, 85.639, 19.575},
                                           {104.897, 82.18, 18.1665},
                                           {26.315, 38.921, 25.5655}};

vector<vector<float>> LM_VERTICES      =  {{-37.93430000, 38.79960000, 45.88450000},
                                           {107.83500000, 60.92790000, 109.70500000},
                                           {83.21620000, 82.65910000, 37.23640000},
                                           {68.32970000, 71.51510000, 50.24850000},
                                           {50.39580000,  90.89790000,  96.86700000},
                                           {33.50540000,  63.81650000,  58.72830000},
                                           {58.78990000,  45.75560000,  47.31120000},
                                           {114.73800000,  37.73570000,  104.00100000},
                                           {52.21460000,  38.70380000,  42.84850000},
                                           {75.09230000,  53.53750000,  34.62070000},
                                           {18.36050000,  38.93300000,  86.40790000},
                                           {50.44390000,  54.24850000,  45.40000000},
                                           {129.11300000,  59.24100000,  70.56620000},
                                           {101.57300000,  58.87630000,  106.55800000},
                                           {46.95910000,  73.71670000,  92.37370000}};

vector<vector<float>> vertices_order    = {{-1, -1, -1},
                                           {-1, -1, 1},
                                           {-1, 1,  1},
                                           {-1, 1, -1},
                                           {1, -1, -1},
                                           {1, -1,  1},
                                           {1,  1,  1},
                                           {1,  1, -1}};

static const string classnames_ycbv[] =
{
    [0] = "master_chef_can",
    [1] = "cracker_box",
    [2] = "sugar_box",
    [3] = "tomato_soup_can",
    [4] = "mustard_bottle",
    [5] = "tuna_fish_can",
    [6] = "pudding_box",
    [7] = "gelatin_box",
    [8] = "potted_meat_can",
    [9] = "banana",
    [10] = "pitcher_base",
    [11] = "bleach_cleanser",
    [12] = "bowl",
    [13] = "mug",
    [14] = "power_drill",
    [15] = "wood_block",
    [16] = "scissors",
    [17] = "large_marker",
    [18] = "large_clamp",
    [19] = "extra_large_clamp",
    [20] = "foam_brick",
};