#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCLを使うときに必要なやつ
// #include <pcl/ros/conversions.h> // 現在は使えない
#include <pcl_conversions/pcl_conversions.h> // 代わりにこれを使う

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

ros::Publisher pub;

//void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  /*/
  // pclをrosで扱える形式に変換
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);

  pcl::ModelCoefficients coefficients; //coefficient;係数
  pcl::PointIndices inliers;

  // Create segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  // Optional
  seg.setOptimizeCoefficients(true);

  // Mandatory; 必須
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);

  seg.setInputCloud(cloud.makeShared());
  seg.segment(inliers, coefficients);

  sensor_msgs::PointCloud2 output;

  // publish the data
  //pub.publish(output);
  pub.publish(coefficients);
  /*/
  // Container for original & filtered data
 pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
 pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
 pcl::PCLPointCloud2 cloud_filtered;

 // Convert to PCL data type
 pcl_conversions::toPCL(*cloud_msg, *cloud);

 // Perform the actual filtering
 pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
 sor.setInputCloud (cloudPtr);
 sor.setLeafSize (0.1, 0.1, 0.1);
 sor.filter (cloud_filtered);

 // Convert to ROS data type
 sensor_msgs::PointCloud2 output;
 pcl_conversions::fromPCL(cloud_filtered, output);

 // Publish the data
 pub.publish (output);

} // cloud_cb

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_tutorial");
  ros::NodeHandle nh;

  // Create Subscriber for input PointCloud
  ros::Subscriber sub;
  sub = nh.subscribe("input", 1, cloud_cb);

  // Create Publisher for output PointCloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
  //pub = nh.advertise<pcl::ModelCoefficients>("output", 1);

  ros::spin();
} // main
