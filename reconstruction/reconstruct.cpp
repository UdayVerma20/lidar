#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/filter.h>
#include </home/uday/catkin_ws/devel/include/clustering/CoordinateList.h>
#include </home/uday/catkin_ws/devel/include/clustering/Coordinates.h>
#include<bits/stdc++.h>
#include <pcl_ros/point_cloud.h>

using namespace std;

#define lidarframe "rslidar"
#define heightcuboid 0.35
#define lengthcuboid 0.1 //front and back of cone
#define widthcuboid 0.2 //side to side of cone

vector<vector<float>> clusters;
ros::Publisher pub;

void 
cluster_cb (clustering::CoordinateList inputclusters)
{
  clusters.clear();
  for(auto i:inputclusters.ConeCoordinates){
    clusters.push_back({i.x, i.y, i.z});
  }
}

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr reconstructedcloud(new pcl::PointCloud<pcl::PointXYZI>);
  reconstructedcloud->header.frame_id = lidarframe;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudtomanipulate(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg (*input, *cloudtomanipulate);
  for(int index=0; index<cloudtomanipulate->points.size(); index++){
    for(auto i:clusters){
        if((cloudtomanipulate->points)[index].x >= i[0] - lengthcuboid && (cloudtomanipulate->points)[index].x <= i[0] + lengthcuboid
        && (cloudtomanipulate->points)[index].y >= i[1] - widthcuboid && (cloudtomanipulate->points)[index].y <= i[1] + widthcuboid
        && (cloudtomanipulate->points)[index].z >= i[2] - heightcuboid && (cloudtomanipulate->points)[index].z <= i[2] + heightcuboid        
        ) reconstructedcloud->push_back((cloudtomanipulate->points)[index]);
    }
  }
  ROS_INFO("Publishing to reconstructedcluster");
  pub.publish (*reconstructedcloud);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reconstruct");
  ros::NodeHandle nh;

  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI>> ("reconstructedcluster", 1);

  ros::Subscriber subcluster = nh.subscribe ("Clusters", 1, cluster_cb);
  ros::Subscriber subcloud = nh.subscribe ("velodyne_points", 1, cloud_cb);
  
  ros::spin();
  return 0;
}