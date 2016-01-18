#ifndef FRONTIERS_H
#define FRONTIERS_H


typedef struct frontier_position {
    int x_mean, y_mean, num, min_dist;

    double window;
    double wave;
    double neg_wave;

    std::vector<int> x;
    std::vector<int> y;
}iposs;


#endif // FRONTIERS_H
