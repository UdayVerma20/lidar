#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include<ground_removal/arr.h>
#include<ground_removal/arrofarr.h>
#include<bits/stdc++.h>
using namespace std;

//parameters
#define heightlidar 0.188 //0.25

#define ringlength 0.2 //1.5
#define startatring 0
#define planespreadbeforering 120
#define lidarframe "velodyne"

//tune-able parameters
#define pnaught 2
#define tau 1
#define distancethreshold 0.03
#define leftmax 10
#define rightmax 10
#define topmax 3.0
float sectorangle = M_PI/48; //sector angle has to divide 2*pi/3 even times

int markerid = 0;
ros::Publisher non_ground_pub;
ros::Publisher ground_pub;
ros::Publisher marker_pub;

class ComparePoints {
public:
    bool operator()(pcl::PointXYZI& p1, pcl::PointXYZI& p2)
    {
       if (p1.z < p2.z) return true;
       return false;
    }
};

vector<vector<pcl::PointXYZI>> visualisationlist;
unordered_map<int, priority_queue<pcl::PointXYZI, vector<pcl::PointXYZI>, ComparePoints>> bin_max_points;
std::unordered_map<int, pcl::PointXYZI> bin_min_points;
vector<vector<float>> ground(static_cast<int>(round(M_PI/(sectorangle)))*planespreadbeforering, vector<float>(4, 0));
pcl::PointXYZI lidarpoint;

ground_removal::arrofarr ground_msg ;
ground_removal::arr temp;

void
storeData (int bin, float a, float b, float c, float d){
  ground[bin] = {a, b, c, d};
  ground[bin+1] = {a, b, c, d};
  temp.data = {float(bin),a,b,c,d};
  ground_msg.data.push_back(temp);
  temp.data = {float(bin+1),a,b,c,d};
  ground_msg.data.push_back(temp);
}

void visualise(){
  visualization_msgs::MarkerArray visualisationarray;
  //making rings
  for(int i=0; i<planespreadbeforering; i++){
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
    for(int j=0; j<19; j++){
      geometry_msgs::Point ringpoint;
      ringpoint.x = i*ringlength*sin(j*M_PI/18);
      ringpoint.y = -i*ringlength*cos(j*M_PI/18);
      ringpoint.z = -heightlidar;
      ringmarker.points.push_back(ringpoint);
    }
    visualisationarray.markers.push_back(ringmarker);
  }

  //making sectors
  for(int i=0; i<=static_cast<int>(round(M_PI/(sectorangle))); i++){
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
    sectorpoint1.x = 0;sectorpoint1.y=0;sectorpoint1.z=-heightlidar;
    ringmarker.points.push_back(sectorpoint1);
    geometry_msgs::Point sectorpoint2;
    sectorpoint2.x = planespreadbeforering*ringlength*sin(i*sectorangle);sectorpoint2.y=-planespreadbeforering*ringlength*cos(i*sectorangle);sectorpoint2.z=-heightlidar;
    ringmarker.points.push_back(sectorpoint2);
    visualisationarray.markers.push_back(ringmarker);
  }

  //making planes
  for(auto total:visualisationlist){
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
    ringmarker.color.g = 1.0;
    ringmarker.color.a = 1.0;

    for(auto i:total){
      geometry_msgs::Point minpoint;
      minpoint.x = i.x;minpoint.y = i.y;minpoint.z = i.z;
      ringmarker.points.push_back(minpoint);
    }
    visualisationarray.markers.push_back(ringmarker);
  }
  ROS_INFO("Visualising to /visualization_marker");
  marker_pub.publish(visualisationarray);
}


void
propogate (pcl::PointXYZI lastpoint, int bin){
  if(int(bin/static_cast<int>(round(M_PI/(sectorangle))))>=planespreadbeforering) return;
  if(bin_min_points.find(bin)==bin_min_points.end() || bin_min_points.find(bin+1)==bin_min_points.end()){
    if(bin_min_points.find(bin)!=bin_min_points.end()){
        storeData(bin, 0.f, 0.f, 1.f, -bin_min_points[bin].z);
        lastpoint = bin_min_points[bin];
    }
    else if(bin_min_points.find(bin+1)!=bin_min_points.end()){
        storeData(bin, 0.f, 0.f, 1.f, -bin_min_points[bin+1].z);
        lastpoint = bin_min_points[bin+1];
    }else{
        storeData(bin, 0.f, 0.f, 1.f, heightlidar);
    }
    propogate(lastpoint, bin+static_cast<int>(round(M_PI/(sectorangle))));
    return;
  }
  float x1 = lastpoint.x;
  float y1 = lastpoint.y;
  float z1 = lastpoint.z;

  float x2 = bin_min_points[bin].x;
  float y2 = bin_min_points[bin].y;
  float z2 = bin_min_points[bin].z;

  float x3 = bin_min_points[bin+1].x;
  float y3 = bin_min_points[bin+1].y;
  float z3 = bin_min_points[bin+1].z;
 
  visualisationlist.push_back({lastpoint, bin_min_points[bin], bin_min_points[bin+1], lastpoint});

  float a = y2*z3 + y3*z1 + y1*z2 - (y1*z3 + y2*z1 + y3*z2);
  float b = x3*z2 + x1*z3 + x2*z1 - (x3*z1 + x1*z2 + x2*z3);
  float c = x2*y3 + x1*y2 + x3*y1 - (x2*y1 + x1*y3 + x3*y2);
  float d = -x1*a - y1*b - z1*c;
  storeData(bin, a, b, c, d);
  lastpoint.x = (x2+x3)/2; lastpoint.y = (y2+y3)/2; lastpoint.z = (z2+z3)/2;
  propogate(lastpoint, bin+static_cast<int>(round(M_PI/(sectorangle))));
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
  bin_max_points.clear();
  ground_msg.data.clear();
  visualisationlist.clear();
  lidarpoint.x = 0.f; lidarpoint.y = 0.f; lidarpoint.z = -heightlidar;

  //segregating points into bins and placing them in bin_max_points
  for(int index=0; index<cloudtomanipulate->points.size(); index++){

    //trimming view area
    if((cloudtomanipulate->points)[index].y >leftmax || (cloudtomanipulate->points)[index].y<-rightmax || (cloudtomanipulate->points)[index].z >topmax) continue;

    int ring = pow(pow((cloudtomanipulate->points)[index].x, 2)+pow((cloudtomanipulate->points)[index].y, 2), 0.5)/ringlength;
    if(ring>=planespreadbeforering) continue;
    float angle = atan2((cloudtomanipulate->points)[index].x, -(cloudtomanipulate->points)[index].y);
    if(angle >=M_PI|| angle <0) continue;

    //this key starts from 0 from right direction and denotes the bin number. It increases as we move towards left or farther away from lidar
    int key = static_cast<int>((angle)/(sectorangle)) + static_cast<int>(round(M_PI/(sectorangle)))*ring;

    //number of points to take avg of
    int maxpoints = floor(pnaught*pow(M_E, -ring/tau))+1;

    //adding to bin_max_points
    if(bin_max_points[key].size()>maxpoints){
        if(bin_max_points[key].top().z > (cloudtomanipulate->points)[index].z){
            bin_max_points[key].pop();
            bin_max_points[key].push((cloudtomanipulate->points)[index]);
        }
    }
    else bin_max_points[key].push((cloudtomanipulate->points)[index]);
  }

  for(auto i:bin_max_points){
    int c=0;
    while(!i.second.empty()){
        if(c==0){
            bin_min_points[i.first] = i.second.top();
            i.second.pop();
            c++;
            continue;
        }
        bin_min_points[i.first].x = (c*bin_min_points[i.first].x + i.second.top().x)/(c+1);
        bin_min_points[i.first].y = (c*bin_min_points[i.first].y + i.second.top().y)/(c+1);
        bin_min_points[i.first].z = (c*bin_min_points[i.first].z + i.second.top().z)/(c+1);
        i.second.pop();
        c++;
    }
  }

  //propogating the planes through all the rings
  for(int i=static_cast<int>(round(M_PI/(sectorangle)))*startatring; i<static_cast<int>(round(M_PI/(sectorangle)))*(startatring+1); i+=2){
    propogate(lidarpoint, i);
   
  }

  visualise();

  //removing points in the viscinity of the planes
  pcl::PointCloud<pcl::PointXYZI>::Ptr nonground(new pcl::PointCloud<pcl::PointXYZI>);
  nonground->header.frame_id = lidarframe;
  for(int index=0; index<cloudtomanipulate->points.size(); index++){

    //trimming view area
    if((cloudtomanipulate->points)[index].y >leftmax || (cloudtomanipulate->points)[index].y<-rightmax || (cloudtomanipulate->points)[index].z >topmax) continue;

    int ring = static_cast<int>(pow(pow((cloudtomanipulate->points)[index].x, 2)+pow((cloudtomanipulate->points)[index].y, 2), 0.5)/ringlength);
    if(ring>=planespreadbeforering) continue;
    float angle = atan2((cloudtomanipulate->points)[index].x, -(cloudtomanipulate->points)[index].y);
    if(angle >=M_PI|| angle <0) continue;

    int key = static_cast<int>((angle)/(sectorangle)) + static_cast<int>(round(M_PI/(sectorangle)))*ring;

    float denominator = pow(pow(ground[key][0], 2) + pow(ground[key][1], 2) + pow(ground[key][2], 2), 0.5);
    float distance = abs((cloudtomanipulate->points)[index].x*ground[key][0] + (cloudtomanipulate->points)[index].y*ground[key][1] + (cloudtomanipulate->points)[index].z*ground[key][2] + ground[key][3])/denominator;

    if(distance>distancethreshold) nonground->push_back((cloudtomanipulate->points)[index]);
  }

  // Publish the data
  ROS_INFO("Publishing to ConeCloud");
  non_ground_pub.publish (*nonground);
  ROS_INFO("Publishing to ground");
  ground_pub.publish (ground_msg);

}

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "lusim");
  ros::NodeHandle nh;
  non_ground_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI>> ("ConeCloud", 1);
  ground_pub = nh.advertise<ground_removal::arrofarr>("ground", 1);
  marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
  ros::Subscriber sub = nh.subscribe ("velodyne_points", 1, cloud_cb);
  ros::spin ();
}
