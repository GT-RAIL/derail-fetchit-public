#include "manipulation_actions/ScoredPose.h"

using namespace std;

ScoredPose::ScoredPose(geometry_msgs::PoseStamped pose, double score)
{
  this->pose = pose;
  this->score = score;
}

bool ScoredPose::operator < (const ScoredPose obj) const
{
  return this->score < obj.score;
}
