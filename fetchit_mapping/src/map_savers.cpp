#include "fetchit_mapping/map_savers.h"

using namespace std;
using namespace octomap;


namespace fetchit_mapping {

    class MapGenerator
    {

      public:
        MapGenerator(const std::string& mapname, int threshold_occupied, int threshold_free)
          : mapname_(mapname), saved_map_(false), threshold_occupied_(threshold_occupied), threshold_free_(threshold_free)
        {
          ros::NodeHandle n;
          ROS_INFO("Waiting for the map");
          map_sub_ = n.subscribe("map", 1, &MapGenerator::mapCallback, this);
        }

        void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
        {
          ROS_INFO("Received a %d X %d map @ %.3f m/pix",
                   map->info.width,
                   map->info.height,
                   map->info.resolution);


          std::string mapdatafile = mapname_ + ".pgm";
          ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
          FILE* out = fopen(mapdatafile.c_str(), "w");
          if (!out)
          {
            ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
            return;
          }

          fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
                  map->info.resolution, map->info.width, map->info.height);
          for(unsigned int y = 0; y < map->info.height; y++) {
            for(unsigned int x = 0; x < map->info.width; x++) {
              unsigned int i = x + (map->info.height - y - 1) * map->info.width;
              if (map->data[i] >= 0 && map->data[i] <= threshold_free_) { // [0,free)
                fputc(254, out);
              } else if (map->data[i] >= threshold_occupied_) { // (occ,255]
                fputc(000, out);
              } else { //occ [0.25,0.65]
                fputc(205, out);
              }
            }
          }

          fclose(out);


          std::string mapmetadatafile = mapname_ + ".yaml";
          ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
          FILE* yaml = fopen(mapmetadatafile.c_str(), "w");


          /*
    resolution: 0.100000
    origin: [0.000000, 0.000000, 0.000000]
    #
    negate: 0
    occupied_thresh: 0.65
    free_thresh: 0.196
           */

          geometry_msgs::Quaternion orientation = map->info.origin.orientation;
          tf2::Matrix3x3 mat(tf2::Quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
          ));
          double yaw, pitch, roll;
          mat.getEulerYPR(yaw, pitch, roll);

          fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
                  mapdatafile.c_str(), map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, yaw);

          fclose(yaml);

          ROS_INFO("Done\n");
          saved_map_ = true;
        }

        std::string mapname_;
        ros::Subscriber map_sub_;
        bool saved_map_;
        int threshold_occupied_;
        int threshold_free_;

    };

    class MapSaver{
    public:
      MapSaver(const std::string& mapname, bool full){
        ros::NodeHandle n;
        std::string servname = "octomap_binary";
        if (full)
          servname = "octomap_full";
        ROS_INFO("Requesting the map from %s...", n.resolveName(servname).c_str());
        octomap_msgs::GetOctomap::Request req;
        octomap_msgs::GetOctomap::Response resp;
        while(n.ok() && !ros::service::call(servname, req, resp))
        {
          ROS_WARN("Request to %s failed; trying again...", n.resolveName(servname).c_str());
          usleep(1000000);
        }

        if (n.ok()){ // skip when CTRL-C

          AbstractOcTree* tree = octomap_msgs::msgToMap(resp.map);
          AbstractOccupancyOcTree* octree = NULL;
          if (tree){
            octree = dynamic_cast<AbstractOccupancyOcTree*>(tree);
          } else {
            ROS_ERROR("Error creating octree from received message");
            if (resp.map.id == "ColorOcTree")
              ROS_WARN("You requested a binary map for a ColorOcTree - this is currently not supported. Please add -f to request a full map");
          }

          if (octree){
            ROS_INFO("Map received (%zu nodes, %f m res), saving to %s", octree->size(), octree->getResolution(), mapname.c_str());

            std::string suffix = mapname.substr(mapname.length()-3, 3);
            if (suffix== ".bt"){ // write to binary file:
              if (!octree->writeBinary(mapname)){
                ROS_ERROR("Error writing to file %s", mapname.c_str());
              }
            } else if (suffix == ".ot"){ // write to full .ot file:
              if (!octree->write(mapname)){
                ROS_ERROR("Error writing to file %s", mapname.c_str());
              }
            } else{
              ROS_ERROR("Unknown file extension, must be either .bt or .ot");
            }


          } else{
            ROS_ERROR("Error reading OcTree from stream");
          }

          delete octree;

        }
      }
    };
}
