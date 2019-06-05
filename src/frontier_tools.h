#ifndef FRONTIER_TOOLS_H
#define FRONTIER_TOOLS_H

#include <vector>

typedef struct frontier_data{
    int x_mean, y_mean, num, min_dist;
    std::vector<int> x;
    std::vector<int> y;
    unsigned int center;
}iposs;

#endif // FRONTIER_TOOLS_H