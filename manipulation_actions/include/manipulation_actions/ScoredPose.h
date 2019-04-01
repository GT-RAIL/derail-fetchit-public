#ifndef MANIPULATION_ACTIONS_SCORED_POSE_H
#define MANIPULATION_ACTIONS_SCORED_POSE_H

// ROS
#include <geometry_msgs/PoseStamped.h>

class ScoredPose
{
public:
  geometry_msgs::PoseStamped pose;
  double score;

  ScoredPose(geometry_msgs::PoseStamped pose, double score);

  bool operator < (const ScoredPose obj) const;
};

#endif  // MANIPULATION_ACTIONS_SCORED_POSE_H
