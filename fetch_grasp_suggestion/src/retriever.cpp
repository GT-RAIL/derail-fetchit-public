#include <fetch_grasp_suggestion/retriever.h>

using std::ios;
using std::string;
using std::stringstream;
using std::vector;


Retriever::Retriever() :
  pn_("~")
{
  pn_.param<double>("min_grasp_depth", min_grasp_depth_, -0.03);
  pn_.param<double>("max_grasp_depth", max_grasp_depth_, 0.03);

  retrieve_grasps_service_ = pn_.advertiseService("retrieve_grasps", &Retriever::retrieveGraspsCallback, this);
}


bool Retriever::retrieveGraspsCallback(fetch_grasp_suggestion::RetrieveGrasps::Request &req,
    fetch_grasp_suggestion::RetrieveGrasps::Response &res)
{
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "retriever");

  Retriever r;

  ros::spin();

  return EXIT_SUCCESS;
}
