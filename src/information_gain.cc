#include "pioneer3at/OccMap.h"

int count_maps = 0;

void start_map_inf(double & i){
    i=0.0;
}

void save_info_map(pioneer3at::OccMap *map, std::vector<double> inf)
{
    ROS_INFO("Received a %d X %d map @ %.3f m/pix",
            map->map.info.width,
            map->map.info.height,
            map->map.info.resolution);
    char sysCall[512];
    std::string package = ros::package::getPath("pioneer3at");
    std::string mapdatafile = package + "/maps/information_" + boost::to_string(count_maps++);
    sprintf(sysCall, "%s.ppm", mapdatafile.c_str());
    FILE* printFile = fopen(sysCall, "w");
    fprintf(printFile, "P6\n # particles.ppm \n %d %d\n", map->map.info.width, map->map.info.height);
    fprintf(printFile, "255\n");

    for(unsigned int y = 0; y < map->map.info.height; y++) {
        for(unsigned int x = 0; x < map->map.info.width; x++) {
            unsigned int i = x + (map->map.info.height - y - 1) * map->map.info.width;
            fprintf(printFile, "%c%c%c", (int)(inf[i]/2), 0, 255 - (int)(inf[i]/2));
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

std::vector<double> calculate_information_map(pioneer3at::OccMap *map, int n_size, double sigma){
    int height = map->map.info.height;
    int width = map->map.info.width;
    double f;
    std::vector<double>  inf;
    std::vector<double>  aux;
    double max = 0.0;
    double min = 9999999.9;
    double x_mean = 0.0;
    inf=map->data;
    for_each(inf.begin(), inf.end(), start_map_inf);

    for(unsigned int y = n_size; y < height-n_size; y++) {
        for(unsigned int x = n_size; x < width-n_size; x++) {
            unsigned int i = x + (height - y - 1) * width;
            f = 0.0;
            for(unsigned int k = y-n_size; k <= y+n_size; k++){
                for(unsigned int j = x-n_size; j <= x+n_size;j ++){
                    unsigned int e = j + (height - k - 1) * width;
                    
                    if (map->data[e] == -1)
                        x_mean = 0.5;
                    else 
                        x_mean = map->data[e];
                    f += exp(-0.5 * pow((x_mean - 0.5) / sigma, 2));
                    
                }
            }
            inf[i] = f;
            if (f > max){
                max = f;
            }
            if (f < min){
                min = f;
            }
        }
    }

    for (unsigned int i = 0; i < inf.size(); i++){
        inf[i] = (inf[i]-min)/(max-min)*510;
    }

    save_info_map(map, inf);
    
    return inf;

}

