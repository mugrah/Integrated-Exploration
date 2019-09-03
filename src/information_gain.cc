#include "pioneer3at/OccMap.h"


pioneer3at::OccMap calculate_information_map(pioneer3at::OccMap map){

    int height = map.map.info.height;
    int width = map.map.info.width;

    for(unsigned int y = 0; y < height; y++) {
        for(unsigned int x = 0; x < width; x++) {
            unsigned int i = x + (height - y - 1) * width;
        }
    }

}
