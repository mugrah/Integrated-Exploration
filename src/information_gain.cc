#include "gmapping/occMap.h"


gmapping::occMap calculate_information_map(gmapping::occMap map){

    int height = map.map.info.height;
    int width = map.map.info.width;

    for(unsigned int y = 0; y < height; y++) {
        for(unsigned int x = 0; x < width; x++) {
            unsigned int i = x + (height - y - 1) * width;
        }
    }

}
