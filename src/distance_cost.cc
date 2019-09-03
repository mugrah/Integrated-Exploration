#include "gmapping/OccMap.h"
#include "utils.h"

#define MAP_FREE 99999.0

int count_wave = 0;

void save_map_wave(pioneer3at::OccMap map, std::vector<double>  aux)
{
    ROS_INFO("Received a %d X %d map @ %.3f m/pix",
             map.map.info.width,
             map.map.info.height,
             map.map.info.resolution);
    char sysCall[512];

    std::string mapdatafile = "/home/alves/ros_workspaces/poc_ws/src/pioneer3at/maps/wave_" + boost::to_string(count_wave++);
    sprintf(sysCall, "%s.ppm", mapdatafile.c_str());
    FILE* printFile = fopen(sysCall, "w");
    fprintf(printFile, "P6\n # particles.ppm \n %d %d\n", map.map.info.width, map.map.info.height);
    fprintf(printFile, "255\n");

    for(unsigned int y = 0; y < map.map.info.height; y++) {
        for(unsigned int x = 0; x < map.map.info.width; x++) {
            unsigned int i = x + (map.map.info.height - y - 1) * map.map.info.width;
            if(aux[i] ==-1)
              fprintf(printFile, "%c%c%c", 0, 0, 0);
            else
              fprintf(printFile, "%c%c%c", (int)aux[i], 0, 0);
            
        }
    }
    fclose(printFile);
    sprintf(sysCall, "convert %s.ppm %s.png", mapdatafile.c_str(), mapdatafile.c_str());
    system(sysCall);
    sprintf(sysCall, "chmod 666 %s.ppm", mapdatafile.c_str());
    system(sysCall);
    sprintf(sysCall, "chmod 666 %s.png", mapdatafile.c_str());
    system(sysCall);
    sprintf(sysCall, "rm %s.ppm", mapdatafile.c_str());
    system(sysCall);


    ROS_INFO("Done\n");
}

void start_map(double & i){
    if(i == 0.0)
        i=MAP_FREE;
    else
        i=-1.0;
}

std::vector<double> wavefront(pioneer3at::OccMap *map, mapPose m_pose, double &max){

    std::vector<double>  aux;

    int height = map->map.info.height;
    int width = map->map.info.width;
    aux=map->data;
    for_each(aux.begin(), aux.end(), start_map);

    unsigned int ir;
    for(unsigned int y = m_pose.y - 5; y <= m_pose.y + 5; y++){
        for(unsigned int x = m_pose.x -5; x <= m_pose.x +5; x++){
            ir = x + (height - y - 1) * width;
            // if(aux[ir] == MAP_FREE){
                aux[ir] = 255.0;
            // }
        }
    }

    save_map_wave(*map, aux);

    // unsigned int ir;
    for(unsigned int y = m_pose.y - 5; y <= m_pose.y + 5; y++){
        for(unsigned int x = m_pose.x -5; x <= m_pose.x +5; x++){
            ir = x + (height - y - 1) * width;
            if(aux[ir] == 255.0){
                aux[ir] = 0.0;
            }
        }
    }

    
    
    int changes = 1;
    unsigned int ic,it;
    while(changes){
        changes = 0;
        for(unsigned int y = 1; y < height; y++) {
            for(unsigned int x = 1; x < width; x++) {

                ic = x + (height - y - 1) * width;
                if(aux[ic] != MAP_FREE && aux[ic] != -1.0){

                    it = x-1 + (height - y-1 - 1) * width;
                    if(aux[it]==MAP_FREE){
                        aux[it]=aux[ic]+1;
                        if(aux[it]>max)
                            max = aux[it];
                        changes = 1;
                    }

                    it = x-1 + (height - y - 1) * width;
                    if(aux[it]==MAP_FREE){
                        aux[it]=aux[ic]+1;
                        if(aux[it]>max)
                            max = aux[it];
                        changes = 1;
                    }

                    it = x-1 + (height - y+1 - 1) * width;
                    if(aux[it]==MAP_FREE){
                        aux[it]=aux[ic]+1;
                        if(aux[it]>max)
                            max = aux[it];
                        changes = 1;
                    }

                    it = x + (height - y-1 - 1) * width;
                    if(aux[it]==MAP_FREE){
                        aux[it]=aux[ic]+1;
                        if(aux[it]>max)
                            max = aux[it];
                        changes = 1;
                    }

                    it = x + (height - y+1 - 1) * width;
                    if(aux[it]==MAP_FREE){
                        aux[it]=aux[ic]+1;
                        if(aux[it]>max)
                            max = aux[it];
                        changes = 1;
                    }

                    it = x+1 + (height - y-1 - 1) * width;
                    if(aux[it]==MAP_FREE){
                        aux[it]=aux[ic]+1;
                        if(aux[it]>max)
                            max = aux[it];
                        changes = 1;
                    }

                    it = x+1 + (height - y - 1) * width;
                    if(aux[it]==MAP_FREE){
                        aux[it]=aux[ic]+1;
                        if(aux[it]>max)
                            max = aux[it];
                        changes = 1;
                    }

                    it = x+1 + (height - y+1 - 1) * width;
                    if(aux[it]==MAP_FREE){
                        aux[it]=aux[ic]+1;
                        if(aux[it]>max)
                            max = aux[it];
                        changes = 1;
                    }

                }
            }
        }
        for(unsigned int y = height-1; y > 0; y--) {
            for(unsigned int x = width-1; x >0 ; x--) {

                ic = x + (height - y - 1) * width;
                if(aux[ic] != MAP_FREE && aux[ic] != -1.0){

                    it = x-1 + (height - y-1 - 1) * width;
                    if(aux[it]==MAP_FREE){
                        aux[it]=aux[ic]+1;
                        if(aux[it]>max)
                            max = aux[it];
                        changes = 1;
                    }

                    it = x-1 + (height - y - 1) * width;
                    if(aux[it]==MAP_FREE){
                        aux[it]=aux[ic]+1;
                        if(aux[it]>max)
                            max = aux[it];
                        changes = 1;
                    }

                    it = x-1 + (height - y+1 - 1) * width;
                    if(aux[it]==MAP_FREE){
                        aux[it]=aux[ic]+1;
                        if(aux[it]>max)
                            max = aux[it];
                        changes = 1;
                    }

                    it = x + (height - y-1 - 1) * width;
                    if(aux[it]==MAP_FREE){
                        aux[it]=aux[ic]+1;
                        if(aux[it]>max)
                            max = aux[it];
                        changes = 1;
                    }

                    it = x + (height - y+1 - 1) * width;
                    if(aux[it]==MAP_FREE){
                        aux[it]=aux[ic]+1;
                        if(aux[it]>max)
                            max = aux[it];
                        changes = 1;
                    }

                    it = x+1 + (height - y-1 - 1) * width;
                    if(aux[it]==MAP_FREE){
                        aux[it]=aux[ic]+1;
                        if(aux[it]>max)
                            max = aux[it];
                        changes = 1;
                    }

                    it = x+1 + (height - y - 1) * width;
                    if(aux[it]==MAP_FREE){
                        aux[it]=aux[ic]+1;
                        if(aux[it]>max)
                            max = aux[it];
                        changes = 1;
                    }

                    it = x+1 + (height - y+1 - 1) * width;
                    if(aux[it]==MAP_FREE){
                        aux[it]=aux[ic]+1;
                        if(aux[it]>max)
                            max = aux[it];
                        changes = 1;
                    }

                }

            }
        }
    }
    save_map_wave(*map, aux);
    return aux;
}

std::vector<double> calculate_cost_map(pioneer3at::OccMap *map, mapPose m_pose){

    int height = map->map.info.height;
    int width = map->map.info.width;

    double max=0.0;
    std::vector<double> wave = wavefront(map, m_pose, max);
    std::vector<double> norm_wave;
    norm_wave.assign(wave.size(), 0.0);
    for(unsigned int y = 0; y < height; y++) {
        for(unsigned int x = 0; x < width; x++) {
            unsigned int i = x + (height - y - 1) * width;
            if(wave[i]!=MAP_FREE && wave[i]!=-1)
                norm_wave[i] = wave[i]/max;
            else
                norm_wave[i] = -1.0;
        }
    }
    return norm_wave;
}
