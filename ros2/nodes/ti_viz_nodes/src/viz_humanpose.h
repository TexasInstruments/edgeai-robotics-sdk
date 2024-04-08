#include <vector>
#include <string>
using namespace std;

vector<vector<int>> CLASS_COLOR_MAP = {{0, 0, 255}, {255, 0, 0},
                                       {0, 255, 0}, {255, 0, 255},
                                       {0, 255, 255}, {255, 255, 0}};

vector<vector<int>> palette = {{255, 128, 0}, {255, 153, 51},
                               {255, 178, 102}, {230, 230, 0},
                               {255, 153, 255}, {153, 204, 255},
                               {255, 102, 255}, {255, 51, 255},
                               {102, 178, 255}, {51, 153, 255},
                               {255, 153, 153}, {255, 102, 102},
                               {255, 51, 51}, {153, 255, 153},
                               {102, 255, 102}, {51, 255, 51},
                               {0, 255, 0}, {0, 0, 255},
                               {255, 0, 0}, {255, 255, 255}};

vector<vector<int>> skeleton = {{16, 14}, {14, 12}, {17, 15}, {15, 13},
                                {12, 13}, {6, 12}, {7, 13}, {6, 7}, {6, 8},
                                {7, 9}, {8, 10}, {9, 11}, {2, 3}, {1, 2},
                                {1, 3}, {2, 4}, {3, 5}, {4, 6}, {5, 7}};


vector<vector<int>> pose_limb_color = {palette[9], palette[9], palette[9],
                                       palette[9], palette[7], palette[7],
                                       palette[7], palette[0], palette[0],
                                       palette[0], palette[0], palette[0],
                                       palette[16], palette[16], palette[16],
                                       palette[16], palette[16], palette[16],
                                       palette[16]};


vector<vector<int>> pose_kpt_color = {palette[16], palette[16], palette[16],
                                      palette[16], palette[16], palette[0],
                                      palette[0], palette[0], palette[0],
                                      palette[0], palette[0], palette[9],
                                      palette[9], palette[9], palette[9],
                                      palette[9], palette[9]};

int radius = 5;
int steps = 3;
