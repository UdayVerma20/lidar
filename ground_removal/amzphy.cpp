#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <pcl/filters/filter.h>
#include<ground_removal/arr.h>
#include<ground_removal/arrofarr.h>
#include<bits/stdc++.h>
using namespace std;

#define heightlidar 0.250 //0.188
#define ringlength 0.4 //1.5
#define sectorangle M_PI/768 //sector angle has to be a factor of pi/6
#define startatring 0
#define planespreadtillring 35
#define distancetheshold 0.05 //0.13
#define lidarframe "rslidar" //"rslidar"

#define leftmax 2.5
#define rightmax 1.5
#define topmax 1.0

int markerid = 0;

ros::Publisher pub;
ros::Publisher marker_pub;
ros::Publisher line_pub;
std::unordered_map<int, pcl::PointXYZI> bin_min_points;
std::vector<std::vector<float>> line(2*M_PI/(3*sectorangle), std::vector<float>(3, -1.f));
ground_removal::arrofarr line_msg ;
ground_removal::arr temp;

pcl::PointXYZI lidarpoint;

void
propogate (pcl::PointXYZI lastpoint, int sector, int ringcount, int numberofpoints){
  if(ringcount>=planespreadtillring){
    if(lastpoint.x==lidarpoint.x && lastpoint.y==lidarpoint.y && lastpoint.z==lidarpoint.z){
      lastpoint.x = lidarpoint.x + planespreadtillring*ringlength*cos((sector*sectorangle + (sectorangle/2))+M_PI/6);
      lastpoint.y = lidarpoint.y + planespreadtillring*ringlength*sin((sector*sectorangle + (sectorangle/2))+M_PI/6);
    }
    line[sector] = {lastpoint.x, lastpoint.y, lastpoint.z};
    temp.data = {lastpoint.x, lastpoint.y, lastpoint.z};
    line_msg.data.push_back(temp);
    return;
  }
  if(bin_min_points.find(ringcount*(2*M_PI/(3*sectorangle)) + sector)!=bin_min_points.end()){
    numberofpoints++;
    float avgpointx = ((float)(lastpoint.x*(numberofpoints) + bin_min_points[ringcount*(2*M_PI/(3*sectorangle)) + sector].x))/((float)(numberofpoints + 1));
    float avgpointy = ((float)(lastpoint.y*(numberofpoints) + bin_min_points[ringcount*(2*M_PI/(3*sectorangle)) + sector].y))/((float)(numberofpoints + 1));
    float avgpointz = ((float)(lastpoint.z*(numberofpoints) + bin_min_points[ringcount*(2*M_PI/(3*sectorangle)) + sector].z))/((float)(numberofpoints + 1));
    lastpoint.x = avgpointx; lastpoint.y = avgpointy; lastpoint.z = avgpointz;
  }

  propogate(lastpoint, sector, ringcount+1, numberofpoints);
}

void visualise(){
  visualization_msgs::MarkerArray visualisationarray;
  //making rings
  // for(int i=0; i<planespreadtillring; i++){
  //   visualization_msgs::Marker ringmarker;
  //   ringmarker.header.frame_id = lidarframe;
  //   ringmarker.header.stamp = ros::Time::now();
  //   ringmarker.ns = "basic_shapes";
  //   ringmarker.id = markerid++;
  //   ringmarker.type = visualization_msgs::Marker::LINE_STRIP;
  //   ringmarker.action = visualization_msgs::Marker::ADD;
  //   ringmarker.lifetime = ros::Duration(0.1);

  //   ringmarker.pose.orientation.w=1.0;
  //   ringmarker.scale.x = 0.005;
  //   ringmarker.color.r = 1.0;
  //   ringmarker.color.a = 1.0;
  //   for(int j=0; j<13; j++){
  //     geometry_msgs::Point ringpoint;
  //     ringpoint.x = i*ringlength*sin(j*M_PI/18 + M_PI/6);
  //     ringpoint.y = -i*ringlength*cos(j*M_PI/18 + M_PI/6);
  //     ringpoint.z = -heightlidar;
  //     ringmarker.points.push_back(ringpoint);
  //   }
  //   visualisationarray.markers.push_back(ringmarker);
  // }

  // //making sectors
  // for(int i=0; i<=2*M_PI/(3*sectorangle); i++){
  //   visualization_msgs::Marker ringmarker;
  //   ringmarker.header.frame_id = lidarframe;
  //   ringmarker.header.stamp = ros::Time::now();
  //   ringmarker.ns = "basic_shapes";
  //   ringmarker.id = markerid++;
  //   ringmarker.type = visualization_msgs::Marker::LINE_STRIP;
  //   ringmarker.action = visualization_msgs::Marker::ADD;
  //   ringmarker.lifetime = ros::Duration(0.1);

  //   ringmarker.pose.orientation.w=1.0;
  //   ringmarker.scale.x = 0.005;
  //   ringmarker.color.r = 1.0;
  //   ringmarker.color.a = 1.0;
   
  //   geometry_msgs::Point sectorpoint1;
  //   sectorpoint1.x = 0;sectorpoint1.y=0;sectorpoint1.z=-heightlidar;
  //   ringmarker.points.push_back(sectorpoint1);
  //   geometry_msgs::Point sectorpoint2;
  //   sectorpoint2.x = planespreadtillring*ringlength*sin(i*sectorangle + M_PI/6);sectorpoint2.y=-planespreadtillring*ringlength*cos(i*sectorangle + M_PI/6);sectorpoint2.z=-heightlidar;
  //   ringmarker.points.push_back(sectorpoint2);
  //   visualisationarray.markers.push_back(ringmarker);
  // }

  // making plane lines
  for(int i=0; i<2*M_PI/(3*sectorangle); i++){
    visualization_msgs::Marker ringmarker;
    ringmarker.header.frame_id = lidarframe;
    ringmarker.header.stamp = ros::Time::now();
    ringmarker.ns = "basic_shapes";
    ringmarker.id = markerid++;
    ringmarker.type = visualization_msgs::Marker::LINE_STRIP;
    ringmarker.action = visualization_msgs::Marker::ADD;
    ringmarker.lifetime = ros::Duration(0.1);

    ringmarker.pose.orientation.w=1.0;
    ringmarker.scale.x = 0.005;
    ringmarker.color.r = 1.0;
    ringmarker.color.a = 1.0;
   
    geometry_msgs::Point sectorpoint1;
    sectorpoint1.x = lidarpoint.x; sectorpoint1.y=lidarpoint.y; sectorpoint1.z=lidarpoint.z;
    ringmarker.points.push_back(sectorpoint1);
    geometry_msgs::Point sectorpoint2;
    sectorpoint2.x = planespreadtillring*ringlength*(line[i][0] - lidarpoint.x) + lidarpoint.x ;sectorpoint2.y= planespreadtillring*ringlength*(line[i][1] - lidarpoint.y) + lidarpoint.y  ; planespreadtillring*ringlength*(line[i][2] - lidarpoint.z) + lidarpoint.z ;
    ringmarker.points.push_back(sectorpoint2);
    visualisationarray.markers.push_back(ringmarker);
  }

  ROS_INFO("Visualising to /visualization_marker");
  marker_pub.publish(visualisationarray);
}

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudtomanipulate(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg (*input, *cloud);

  pcl::Indices indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloudtomanipulate, indices);
    
  bin_min_points.clear();
  line_msg.data.clear();
  lidarpoint.x = 0.f; lidarpoint.y = 0.f; lidarpoint.z = -heightlidar;

  //segregating points into bins and placing them in bin_min_points and bin_all_points
  for(int index=0; index<cloudtomanipulate->points.size(); index++){

    //trimming view area
    if((cloudtomanipulate->points)[index].y >leftmax || (cloudtomanipulate->points)[index].y<-rightmax) continue;
    if((cloudtomanipulate->points)[index].z >topmax) continue;

    int ring = pow(pow((cloudtomanipulate->points)[index].x, 2)+pow((cloudtomanipulate->points)[index].y, 2), 0.5)/ringlength;
    if(ring>=planespreadtillring) continue;
    float angle = atan2((cloudtomanipulate->points)[index].x, -(cloudtomanipulate->points)[index].y) - M_PI/6;
    if(angle >=2*M_PI/3|| angle <0) continue;

    //this key starts from 0 from right direction and denotes the bin number. It increases as we move towards left or farther away from lidar
    int key = int((angle)/(sectorangle)) + int((2*M_PI*ring)/(3*sectorangle));

    //adding to bin_min_points
    if(bin_min_points.find(key)!=bin_min_points.end()){
      if((cloudtomanipulate->points)[index].z<bin_min_points[key].z){
    bin_min_points[key] = (cloudtomanipulate->points)[index];
      }
    }else{
      bin_min_points[key] = (cloudtomanipulate->points)[index];
    }
  }
    
  //propogating the planes in all the sectors
  for(int i=int(2*M_PI*startatring/(3*sectorangle)); i<int(2*M_PI*(startatring+1)/(3*sectorangle)); i++){
    propogate(lidarpoint, i, startatring, 0);
  }

  visualise();

  //removing points in the viscinity of the lines
  pcl::PointCloud<pcl::PointXYZI>::Ptr nonground(new pcl::PointCloud<pcl::PointXYZI>);
  nonground->header.frame_id = lidarframe;
  for(int index=0; index<cloudtomanipulate->points.size(); index++){

    //trimming view area
    if((cloudtomanipulate->points)[index].y > leftmax || (cloudtomanipulate->points)[index].y<-rightmax) continue;
    if((cloudtomanipulate->points)[index].z > topmax) continue;

    int ring = pow(pow((cloudtomanipulate->points)[index].x, 2)+pow((cloudtomanipulate->points)[index].y, 2), 0.5)/ringlength;
    if(ring>=planespreadtillring) continue;
    float angle = atan2((cloudtomanipulate->points)[index].x, -(cloudtomanipulate->points)[index].y) - M_PI/6;
    if(angle >=2*M_PI/3|| angle <0) continue;

    int key = int((angle)/(sectorangle));
    if(key>=line.size() || key<0) continue;

    float denominator = pow(pow(lidarpoint.x-line[key][0], 2) + pow(lidarpoint.y-line[key][1], 2) + pow(lidarpoint.z-line[key][2], 2), 0.5);
    float term1 = pow((line[key][1] - lidarpoint.y)*((cloudtomanipulate->points)[index].z - lidarpoint.z) - (line[key][2]-lidarpoint.z)*((cloudtomanipulate->points)[index].y-lidarpoint.y), 2);
    float term2 = pow((line[key][2] - lidarpoint.z)*((cloudtomanipulate->points)[index].x - lidarpoint.x) - (line[key][0]-lidarpoint.x)*((cloudtomanipulate->points)[index].z-lidarpoint.z), 2);
    float term3 = pow((line[key][0] - lidarpoint.x)*((cloudtomanipulate->points)[index].y - lidarpoint.y) - (line[key][1]-lidarpoint.y)*((cloudtomanipulate->points)[index].x-lidarpoint.x), 2);
    float distance = (pow(term1 + term2 + term3, 0.5))/(denominator);

    if(distance>distancetheshold) nonground->push_back((cloudtomanipulate->points)[index]);

  }

  // Publish the data
  ROS_INFO("Publishing to nogroundcloud");
  pub.publish (*nonground);
  ROS_INFO("Publishing to line");
  line_pub.publish (line_msg);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "amzphy");
  ros::NodeHandle nh;

  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI>> ("nogroundcloud", 1);
  marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
  line_pub = nh.advertise<ground_removal::arrofarr>("line", 1);
  
  ros::Subscriber sub = nh.subscribe ("rslidar_points", 1, cloud_cb);
  ros::spin();
  return 0;
}