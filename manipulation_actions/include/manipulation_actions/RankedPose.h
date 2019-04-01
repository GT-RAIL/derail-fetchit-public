#ifndef MANIPULATION_ACTIONS_RANKED_POSE_H
#define MANIPULATION_ACTIONS_RANKED_POSE_H

// ROS
#include <geometry_msgs/PoseStamped.h>

class RankedPose
{
public:
  geometry_msgs::PoseStamped pose;
  double score;
  std::vector<double> hComponents;

  PoseWithHeuristic(geometry_msgs::Pose pose, double h);

  PoseWithHeuristic(geometry_msgs::Pose pose, double h, std::vector<double> hComponents);

  bool operator < (const PoseWithHeuristic obj) const;
};

#endif
