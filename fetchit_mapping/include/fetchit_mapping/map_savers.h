#include <cstdio>
#include <fstream>

#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap_msgs/GetOctomap.h>


namespace fetchit_mapping {

    class MapGenerator {
      public:
        MapGenerator(const std::string& mapname, int threshold_occupied, int threshold_free);

        void mapCallback(const nav_msgs::OccupancyGridConstPtr& map);

        std::string mapname_;
        ros::Subscriber map_sub_;
        bool saved_map_;
        int threshold_occupied_;
        int threshold_free_;

    };

    class MapSaver{
        public:
            MapSaver(const std::string& mapname, bool full);
    };

}