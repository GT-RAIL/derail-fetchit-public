#include <fetch_grasp_suggestion/common.h>

using std::string;
using std::stringstream;
using std::vector;

string Common::createTrainingInstance(vector<double> object_features, vector<double> hi, vector<double> hj,
  bool positive)
{
  stringstream ss;

  //object features (for model-free method)
  for (size_t n = 0; n < object_features.size(); n ++)
  {
    ss << object_features[n] << ",";
  }

  //difference of grasp features
  for (size_t n = 0; n < hi.size(); n ++)
  {
    double value;
    if (positive)
      value = hi[n] - hj[n];
    else
      value = hj[n] - hi[n];
    ss << value << ",";
  }

  //classification (1 for hi > hj, 0 otherwise)
  if (positive)
    ss << 1;
  else
    ss << 0;

  return ss.str();
}

vector<double> Common::createTrainingVector(std::vector<double> object_features, std::vector<double> hi,
  std::vector<double> hj)
{
  vector<double> training_vector;

  for (size_t n = 0; n < object_features.size(); n ++)
  {
    training_vector.push_back(object_features[n]);
  }

  for (size_t n = 0; n < hi.size(); n ++)
  {
    training_vector.push_back(hi[n] - hj[n]);
  }

  return training_vector;
}

vector<double> Common::calculateObjectFeatures(const sensor_msgs::PointCloud2 &cloud)
{
  vector<double> object_features;

  //convert to pcl point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCLPointCloud2 converter;
  pcl_conversions::toPCL(cloud, converter);
  pcl::fromPCLPointCloud2(converter, *pcl_cloud);

  return calculateObjectFeatures(pcl_cloud);
}

vector<double> Common::calculateObjectFeatures(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
  vector<double> object_features;

  //compute ceiLAB color, sorted bounding box dimensions
//  Eigen::Vector3f rgb, lab;
//  rgb[0] = 0;
//  rgb[1] = 0;
//  rgb[2] = 0;
//  for (size_t i = 0; i < cloud->points.size(); i ++)
//  {
//    rgb[0] += cloud->points[i].r;
//    rgb[1] += cloud->points[i].g;
//    rgb[2] += cloud->points[i].b;
//  }
//  for (int i = 0; i < 3; i ++)
//    rgb[i] /= cloud->points.size() * 255.0f;
//  lab = Common::RGB2Lab(rgb);
//  object_features.push_back(lab[0]);
//  object_features.push_back(lab[1]);
//  object_features.push_back(lab[2]);

  fetch_grasp_suggestion::BoundingBox bounding_box = BoundingBoxCalculator::computeBoundingBox(cloud);
  vector<double> dims;
  dims.push_back(bounding_box.dimensions.x);
  dims.push_back(bounding_box.dimensions.y);
  dims.push_back(bounding_box.dimensions.z);
  std::sort(dims.begin() + 1, dims.end());  // sort so that the "vertical" dimension is first, and the two horizontal
  // dimensions are size-sorted in the second and third positions
  for (size_t i = 0; i < dims.size(); i ++)
    object_features.push_back(dims[i]);

  return object_features;
}

vector<double> Common::calculateLocalFeatures(const sensor_msgs::PointCloud2 &cloud, geometry_msgs::Point point)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr converted_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCLPointCloud2::Ptr temp_cloud(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(cloud, *temp_cloud);
  pcl::fromPCLPointCloud2(*temp_cloud, *converted_cloud);

  // crop to local geometry
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::CropBox<pcl::PointXYZRGB> crop_box;
  Eigen::Vector4f min_point, max_point;
  min_point[0] = static_cast<float>(point.x) - 0.15f;
  min_point[1] = static_cast<float>(point.y) - 0.15f;
  min_point[2] = static_cast<float>(point.z) - 0.15f;
  max_point[0] = static_cast<float>(point.x) + 0.15f;
  max_point[1] = static_cast<float>(point.y) + 0.15f;
  max_point[2] = static_cast<float>(point.z) + 0.15f;
  crop_box.setMin(min_point);
  crop_box.setMax(max_point);
  crop_box.setInputCloud(converted_cloud);
  crop_box.filter(*cropped_cloud);

  // find closest cluster to center point
  vector<pcl::PointIndices> cluster_indices;
  vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> clusterer;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  kd_tree->setInputCloud(cropped_cloud);
  clusterer.setInputCloud(cropped_cloud);
  clusterer.setClusterTolerance(0.01);
  clusterer.setMinClusterSize(20);
  clusterer.setMaxClusterSize(30000);
  clusterer.setSearchMethod(kd_tree);
  clusterer.extract(cluster_indices);

  double min_cluster_dst = std::numeric_limits<double>::max();
  size_t min_cluster_index = 0;
  size_t current_cluster = 0;
  vector<int> k_indices;
  vector<float> k_sqr_distances;
  k_indices.resize(1);
  k_sqr_distances.resize(1);
  pcl::PointXYZRGB test_point;
  test_point.x = static_cast<float>(point.x);
  test_point.y = static_cast<float>(point.y);
  test_point.z = static_cast<float>(point.z);
  for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it ++)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit ++)
    {
      temp_cluster->points.push_back(cropped_cloud->points[*pit]);
    }
    temp_cluster->width = temp_cluster->points.size();
    temp_cluster->height = 1;
    temp_cluster->is_dense = true;
    clusters.push_back(temp_cluster);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr searchTree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    searchTree->setInputCloud(temp_cluster);
    searchTree->nearestKSearch(test_point, 1, k_indices, k_sqr_distances);
    if (k_sqr_distances[0] < min_cluster_dst)
    {
      min_cluster_dst = k_sqr_distances[0];
      min_cluster_index = current_cluster;
    }

    current_cluster ++;
  }

  if (clusters.empty())
  {
    return calculateObjectFeatures(converted_cloud);
  }

  return calculateObjectFeatures(clusters[min_cluster_index]);
}

Eigen::Vector3f Common::RGB2Lab(const Eigen::Vector3f& colorRGB)
{
  //convert from RGB color space to CIELAB color space, taken and adapted from pcl/registration/gicp6d

  // for sRGB   -> CIEXYZ see http://www.easyrgb.com/index.php?X=MATH&H=02#text2
  // for CIEXYZ -> CIELAB see http://www.easyrgb.com/index.php?X=MATH&H=07#text7

  double r, g, b, x, y, z;

  r = colorRGB[0];
  g = colorRGB[1];
  b = colorRGB[2];

  // linearize sRGB values
  if (r > 0.04045)
    r = pow ( (r + 0.055) / 1.055, 2.4);
  else
    r = r / 12.92;

  if (g > 0.04045)
    g = pow ( (g + 0.055) / 1.055, 2.4);
  else
    g = g / 12.92;

  if (b > 0.04045)
    b = pow ( (b + 0.055) / 1.055, 2.4);
  else
    b = b / 12.92;

  // postponed:
  //    r *= 100.0;
  //    g *= 100.0;
  //    b *= 100.0;

  // linear sRGB -> CIEXYZ
  x = r * 0.4124 + g * 0.3576 + b * 0.1805;
  y = r * 0.2126 + g * 0.7152 + b * 0.0722;
  z = r * 0.0193 + g * 0.1192 + b * 0.9505;

  // *= 100.0 including:
  x /= 0.95047;  //95.047;
  //    y /= 1;//100.000;
  z /= 1.08883;  //108.883;

  // CIEXYZ -> CIELAB
  if (x > 0.008856)
    x = pow (x, 1.0 / 3.0);
  else
    x = 7.787 * x + 16.0 / 116.0;

  if (y > 0.008856)
    y = pow (y, 1.0 / 3.0);
  else
    y = 7.787 * y + 16.0 / 116.0;

  if (z > 0.008856)
    z = pow (z, 1.0 / 3.0);
  else
    z = 7.787 * z + 16.0 / 116.0;

  Eigen::Vector3f color_lab;
  color_lab[0] = static_cast<float> (116.0 * y - 16.0);
  color_lab[1] = static_cast<float> (500.0 * (x - y));
  color_lab[2] = static_cast<float> (200.0 * (y - z));

  return color_lab;
}
