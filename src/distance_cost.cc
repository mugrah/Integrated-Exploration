#include "gmapping/occMap.h"
#include "utils.h"

#define MAP_FREE 99999

void start_map(double & i){
    if(i>=0.0)
        i=MAP_FREE;
    else
        i=-1;
}

std::vector<double> wavefront(gmapping::occMap map, mapPose m_pose, double *max){

    std::vector<double>  aux;

    aux=map.data;

    for_each(aux.data.begin(), aux.data.end(), start_map);

    unsigned int ir = m_pose.x + (height - m_pose.y - 1) * width;
    aux.data[ir] = 0;


    int changes = 1;
    unsigned int ic,it;
    max = 0;

    while(changes){
        changes = 0;
        for(unsigned int y = 1; y < height; y++) {
            for(unsigned int x = 1; x < width; x++) {

                ic = x + (height - y - 1) * width;
                if(aux.data[i] != MAP_FREE && aux.data[i] != -1){

                    it = x-1 + (height - y-1 - 1) * width;
                    if(aux.data[it]==MAP_FREE){
                        aux.data[it]=aux.data[ic]+1;
                        if(aux.data[it]>max)
                            max = aux.data[it];
                        changes = 1;
                    }

                    it = x-1 + (height - y - 1) * width;
                    if(aux.data[it]==MAP_FREE){
                        aux.data[it]=aux.data[ic]+1;
                        if(aux.data[it]>max)
                            max = aux.data[it];
                        changes = 1;
                    }

                    it = x-1 + (height - y+1 - 1) * width;
                    if(aux.data[it]==MAP_FREE){
                        aux.data[it]=aux.data[ic]+1;
                        if(aux.data[it]>max)
                            max = aux.data[it];
                        changes = 1;
                    }

                    it = x + (height - y-1 - 1) * width;
                    if(aux.data[it]==MAP_FREE){
                        aux.data[it]=aux.data[ic]+1;
                        if(aux.data[it]>max)
                            max = aux.data[it];
                        changes = 1;
                    }

                    it = x + (height - y+1 - 1) * width;
                    if(aux.data[it]==MAP_FREE){
                        aux.data[it]=aux.data[ic]+1;
                        if(aux.data[it]>max)
                            max = aux.data[it];
                        changes = 1;
                    }

                    it = x+1 + (height - y-1 - 1) * width;
                    if(aux.data[it]==MAP_FREE){
                        aux.data[it]=aux.data[ic]+1;
                        if(aux.data[it]>max)
                            max = aux.data[it];
                        changes = 1;
                    }

                    it = x+1 + (height - y - 1) * width;
                    if(aux.data[it]==MAP_FREE){
                        aux.data[it]=aux.data[ic]+1;
                        if(aux.data[it]>max)
                            max = aux.data[it];
                        changes = 1;
                    }

                    it = x+1 + (height - y+1 - 1) * width;
                    if(aux.data[it]==MAP_FREE){
                        aux.data[it]=aux.data[ic]+1;
                        if(aux.data[it]>max)
                            max = aux.data[it];
                        changes = 1;
                    }

                }

            }
        }

        for(unsigned int y = height-1; y >= 0; y--) {
            for(unsigned int x = width-1; x >=0 ; x--) {

                ic = x + (height - y - 1) * width;
                if(aux.data[i] != MAP_FREE && aux.data[i] != -1){

                    it = x-1 + (height - y-1 - 1) * width;
                    if(aux.data[it]==MAP_FREE){
                        aux.data[it]=aux.data[ic]+1;
                        if(aux.data[it]>max)
                            max = aux.data[it];
                        changes = 1;
                    }

                    it = x-1 + (height - y - 1) * width;
                    if(aux.data[it]==MAP_FREE){
                        aux.data[it]=aux.data[ic]+1;
                        if(aux.data[it]>max)
                            max = aux.data[it];
                        changes = 1;
                    }

                    it = x-1 + (height - y+1 - 1) * width;
                    if(aux.data[it]==MAP_FREE){
                        aux.data[it]=aux.data[ic]+1;
                        if(aux.data[it]>max)
                            max = aux.data[it];
                        changes = 1;
                    }

                    it = x + (height - y-1 - 1) * width;
                    if(aux.data[it]==MAP_FREE){
                        aux.data[it]=aux.data[ic]+1;
                        if(aux.data[it]>max)
                            max = aux.data[it];
                        changes = 1;
                    }

                    it = x + (height - y+1 - 1) * width;
                    if(aux.data[it]==MAP_FREE){
                        aux.data[it]=aux.data[ic]+1;
                        if(aux.data[it]>max)
                            max = aux.data[it];
                        changes = 1;
                    }

                    it = x+1 + (height - y-1 - 1) * width;
                    if(aux.data[it]==MAP_FREE){
                        aux.data[it]=aux.data[ic]+1;
                        if(aux.data[it]>max)
                            max = aux.data[it];
                        changes = 1;
                    }

                    it = x+1 + (height - y - 1) * width;
                    if(aux.data[it]==MAP_FREE){
                        aux.data[it]=aux.data[ic]+1;
                        if(aux.data[it]>max)
                            max = aux.data[it];
                        changes = 1;
                    }

                    it = x+1 + (height - y+1 - 1) * width;
                    if(aux.data[it]==MAP_FREE){
                        aux.data[it]=aux.data[ic]+1;
                        if(aux.data[it]>max)
                            max = aux.data[it];
                        changes = 1;
                    }

                }

            }
        }
    }

    return aux;
}

std::vector<double> calculate_cost_map(gmapping::occMap map, mapPose m_pose){

    int height = map.map.info.height;
    int width = map.map.info.width;

    double max;

    std::vector<double> wave = wavefront(map, m_pose, max);
    std::vector<double> norm_wave;
    norm_wave.assign(wave.size(), 0.0);

    for(unsigned int y = 0; y < height; y++) {
        for(unsigned int x = 0; x < width; x++) {
            unsigned int i = x + (height - y - 1) * width;
            if(wave.data[i]!=MAP_FREE && wave.data[i]!=-1)
                norm_wave.data[i] = wave.data[i]/max;
        }
    }

    return norm_wave;
}
