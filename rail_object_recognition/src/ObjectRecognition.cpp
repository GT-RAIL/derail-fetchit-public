#include "rail_object_recognition/ObjectRecognition.h"

using namespace std;

ObjectRecognition::ObjectRecognition(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
	ROS_INFO("In the constructor of object recognition class");
	initializeServices();
}

void ObjectRecognition::initializeServices()
{
	ROS_INFO("Initializing Object Detection Services");
	point_cloud_service_ = nh_.advertiseService("recognize_object", &ObjectRecognition::serviceCallback, this);
	parts_classifier_client_ = nh_.serviceClient<rail_object_recognition::PartsQuery>("/rail_object_recognition/classify_parts");
}

bool ObjectRecognition::serviceCallback(rail_object_recognition::ExtractPointCloud::Request &req, rail_object_recognition::ExtractPointCloud::Response &res)
{
	ROS_INFO("Object Recognition Service Callback Activated");
	rail_object_recognition::PartsQuery parts_query_srv;
	int number_of_objects = req.clouds.size();
	for(int i=0; i<number_of_objects; i++)
	{
		vector<double> desc;
		rail_object_recognition::Descriptor desc_message;
		pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(req.clouds[i], *input_cloud);
		pcl::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> esf;
	        esf.setInputCloud (input_cloud);
	        pcl::PointCloud<pcl::ESFSignature640>::Ptr esfSignature (new pcl::PointCloud<pcl::ESFSignature640>);
	        esf.compute(*esfSignature);
		for(int d=0; d<640; d++){
			desc.push_back(esfSignature->points[0].histogram[d]);
		}
		desc_message.descriptor = desc;
      		parts_query_srv.request.descriptors.push_back(desc_message);		  
	}
	if(parts_classifier_client_.call(parts_query_srv)){
			ROS_INFO("Success in determining object classes");
			res.classes_of_parts = parts_query_srv.response.parts_classes;
			return true;					
	}
	else{
		ROS_INFO("Failure in determining object classes");
		return false;	
	}
return false;	
}






