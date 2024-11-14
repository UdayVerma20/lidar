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
#include <vector>
#include <cstdlib>

#include<bits/stdc++.h>
#define heightlidar 0.27 //0.188 
#define ringlength 0.4 //1.5 
#define sectorangle M_PI/12 //sector angle has to be a multiple of pi/6
#define startatring 0
#define planespreadtillring 50
#define distancethreshold 0.06 //0.13
#define lidarframe "rslidar" //"rslidar"

ros::Publisher pub;
ros::Publisher pub_;
ros::Publisher marker_pub;
std::unordered_map<int, pcl::PointXYZI> bin_min_points;
std::unordered_map<int, std::vector<pcl::PointXYZI>> bin_all_points;
std::unordered_map<int, std::vector<float>> plane;
std::vector<std::vector<pcl::PointXYZI>> visualisationlist;

int markerid = 0;


class DBSCAN
{
public:
    DBSCAN(double eps, int minPts) : eps_(eps), minPts_(minPts) {}

    void cluster(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::vector<std::vector<int>>& clusters)
    {
        std::vector<bool> visited(cloud->points.size(), false);
        pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
        kdtree.setInputCloud(cloud);

        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            if (!visited[i])
            {
                std::vector<int> cluster;
                std::vector<int> neighbors;
                if (expandCluster(cloud, kdtree, i, visited, neighbors))
                {
                    cluster.insert(cluster.end(), neighbors.begin(), neighbors.end());
                    clusters.push_back(cluster);
                }
            }
        }
    }

private:
    bool expandCluster(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::KdTreeFLANN<pcl::PointXYZI>& kdtree,
                       int index, std::vector<bool>& visited, std::vector<int>& cluster)
    {
        std::vector<int> neighbors;
        std::vector<float> sqr_distances;
        if (kdtree.radiusSearch(cloud->points[index], eps_, neighbors, sqr_distances) >= minPts_)
        {
            visited[index] = true;
            cluster.push_back(index);

            for (size_t i = 0; i < neighbors.size(); ++i)
            {
                int neighbor_index = neighbors[i];
                if (!visited[neighbor_index])
                {
                    visited[neighbor_index] = true;
                    std::vector<int> sub_neighbors;
                    std::vector<float> sub_sqr_distances;
                    if (kdtree.radiusSearch(cloud->points[neighbor_index], eps_, sub_neighbors, sub_sqr_distances) >= minPts_)
                    {
                        cluster.insert(cluster.end(), sub_neighbors.begin(), sub_neighbors.end());
                    }
                }
            }
            return true;
        }
        return false;
    }

    double eps_;   // Cluster tolerance (radius)
    int minPts_;   // Minimum number of points to form a cluster
};


void
propogate (pcl::PointXYZI lastpoint, int segment1, int segment2, int ringcount){
  if(ringcount>=planespreadtillring) return;
  if(bin_min_points.find(segment1)==bin_min_points.end() || bin_min_points.find(segment2)==bin_min_points.end()){
    if(bin_min_points.find(segment1)!=bin_min_points.end()) plane[segment1] = {0.f, 0.f, 1.f, -bin_min_points[segment1].z};
    else if(bin_min_points.find(segment2)!=bin_min_points.end()) plane[segment1] = {0.f, 0.f, 1.f, -bin_min_points[segment2].z};

    propogate(lastpoint, segment1+2*M_PI/(3*sectorangle), segment2+2*M_PI/(3*sectorangle), ringcount+1);
    return;
  }
  float x1 = lastpoint.x;
  float y1 = lastpoint.y;
  float z1 = lastpoint.z;

  float x2 = bin_min_points[segment1].x;
  float y2 = bin_min_points[segment1].y;
  float z2 = bin_min_points[segment1].z;

  float x3 = bin_min_points[segment2].x;
  float y3 = bin_min_points[segment2].y;
  float z3 = bin_min_points[segment2].z;
  
  visualisationlist.push_back({lastpoint, bin_min_points[segment1], bin_min_points[segment2], lastpoint});

  float a = y2*z3 + y3*z1 + y1*z2 - (y1*z3 + y2*z1 + y3*z2);
  float b = x3*z2 + x1*z3 + x2*z1 - (x3*z1 + x1*z2 + x2*z3);
  float c = x2*y3 + x1*y2 + x3*y1 - (x2*y1 + x1*y3 + x3*y2);
  float d = -x1*a - y1*b - z1*c;
  plane[segment1] = {a, b, c, d};
  lastpoint.x = (x2+x3)/2; lastpoint.y = (y2+y3)/2; lastpoint.z = (z2+z3)/2;
  propogate(lastpoint, segment1+2*M_PI/(3*sectorangle), segment2+2*M_PI/(3*sectorangle), ringcount+1);
}

void visualise(){
  visualization_msgs::MarkerArray visualisationarray;
  //making rings
  for(int i=0; i<planespreadtillring; i++){
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
    for(int j=0; j<13; j++){
      geometry_msgs::Point ringpoint;
      ringpoint.x = i*ringlength*sin(j*M_PI/18 + M_PI/6);
      ringpoint.y = -i*ringlength*cos(j*M_PI/18 + M_PI/6);
      ringpoint.z = -heightlidar;
      ringmarker.points.push_back(ringpoint);
    }
    visualisationarray.markers.push_back(ringmarker);
  }

  //making sectors
  for(int i=0; i<=2*M_PI/(3*sectorangle); i++){
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
    sectorpoint2.x = planespreadtillring*ringlength*sin(i*sectorangle + M_PI/6);sectorpoint2.y=-planespreadtillring*ringlength*cos(i*sectorangle + M_PI/6);sectorpoint2.z=-heightlidar;
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
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudtomanipulate(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg (*input, *cloud);

  pcl::Indices indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloudtomanipulate, indices);
  // cloudtomanipulate->header.frame_id = "rslidar";
  
  bin_min_points.clear();
  bin_all_points.clear();
  plane.clear();
  visualisationlist.clear();

  //segregating points into bins and placing them in bin_min_points and bin_all_points
  for(int index=0; index<cloudtomanipulate->points.size(); index++){
    int distance = pow(pow((cloudtomanipulate->points)[index].x, 2)+pow((cloudtomanipulate->points)[index].y, 2), 0.5)/ringlength;
    if(distance>=planespreadtillring) continue;
    float angle = atan2((cloudtomanipulate->points)[index].x, -(cloudtomanipulate->points)[index].y) - M_PI/6;
    if(angle >=2*M_PI/3|| angle <0) continue;

    //trimming view area
    if((cloudtomanipulate->points)[index].y >2.0 || (cloudtomanipulate->points)[index].y<-2.0) continue;

    //this key starts from 0 from right direction and denotes the bin number. It increases as we move towards left or farther away from lidar
    int key = int((angle)/(sectorangle)) + int((2*M_PI*distance)/(3*sectorangle));

    //adding to bin_all_points
    bin_all_points[key].push_back((cloudtomanipulate->points)[index]);

    //adding to bin_min_points
    if(bin_min_points.find(key)!=bin_min_points.end()){
      if((cloudtomanipulate->points)[index].z<bin_min_points[key].z){
        bin_min_points[key] = (cloudtomanipulate->points)[index];
      }
    }else{
      bin_min_points[key] = (cloudtomanipulate->points)[index];
    }
  }

  //propogating the planes in all the bins
  pcl::PointXYZI lidarpoint;
  lidarpoint.x = 0.f;
  lidarpoint.y = 0.f;
  lidarpoint.z = -heightlidar;
  for(int i=int(2*M_PI*startatring/(3*sectorangle)); i<int(2*M_PI*(startatring+1)/(3*sectorangle)); i+=2){
    propogate(lidarpoint, i, i+1, startatring);
  }

  //visualising
  visualise();

  //removing points in the viscinity of the planes
  std::vector<float> coeffs(4, 0);
  pcl::PointCloud<pcl::PointXYZI>::Ptr nonground(new pcl::PointCloud<pcl::PointXYZI>);
  nonground->header.frame_id = lidarframe;
  for(auto i:bin_all_points){
    // if(i.first>planespreadtillring*8){
    //   continue;
    // }
    if(i.first%2==0){
      coeffs = plane[i.first];
    }else{
      coeffs = plane[i.first-1];
    }
    float denominator = pow(pow(coeffs[0], 2) + pow(coeffs[1], 2) + pow(coeffs[2], 2), 0.5);
    for(auto j:i.second){
      if(abs((coeffs[0]*j.x + coeffs[1]*j.y + coeffs[2]*j.z + coeffs[3])/denominator)>=distancethreshold) nonground->push_back(j);
    }
  }

  // Publish the data.
  ROS_INFO("Publishing to nogroundcloud");
  // ros::Rate rate(4);
  pub.publish (*nonground);
  // rate.sleep();


  // pcl::PointCloud<pcl::PointXYZI>::Ptr cloudbrr(&nonground);
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
        voxel_grid.setInputCloud(nonground);
        voxel_grid.setLeafSize(0.01f, 0.01f, 0.01f);  // Adjust leaf size as necessary
        voxel_grid.filter(*downsampled_cloud);
        // ROS_INFO("Downsampled point cloud, number of points: %zu", downsampled_cloud->points.size());

        // Apply DBSCAN clustering
        DBSCAN dbscan(0.5, 8);  // Increase eps and minPts for larger, more stable clusters
        std::vector<std::vector<int>> clusters;
        dbscan.cluster(downsampled_cloud, clusters);

        if (clusters.empty())
        {
            ROS_WARN("No clusters found.");
            return;
        }
        // std::cout << "hi" << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr centroids_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<float> coeffsclus(4, 0);

        // Compute the centroid of each cluster and add it to the centroids cloud
        int cluster_id = 0;
        for (const auto& cluster : clusters)
        {
            float sum_x = 0.0, sum_y = 0.0, sum_z = 0.0, max_x = -10000.f, min_x = 10000.f, max_y = -10000.f, min_y = 10000.f, max_z = -10000.f, min_z = 10000.f;
            for (const auto& idx : cluster)
            {
                sum_x += downsampled_cloud->points[idx].x;
                if(max_x<downsampled_cloud->points[idx].x) max_x = downsampled_cloud->points[idx].x;
                if(min_x>downsampled_cloud->points[idx].x) min_x = downsampled_cloud->points[idx].x;
                sum_y += downsampled_cloud->points[idx].y;
                if(max_y<downsampled_cloud->points[idx].y) max_y = downsampled_cloud->points[idx].y;
                if(min_y>downsampled_cloud->points[idx].y) min_y = downsampled_cloud->points[idx].y;
                sum_z += downsampled_cloud->points[idx].z;
                if(max_z<downsampled_cloud->points[idx].z) max_z = downsampled_cloud->points[idx].z;
                if(min_z>downsampled_cloud->points[idx].z) min_z = downsampled_cloud->points[idx].z;
            }

            // Compute the centroid of the cluster
            pcl::PointXYZ centroid;
            centroid.x = sum_x / cluster.size();
            centroid.y = sum_y / cluster.size();
            centroid.z = sum_z / cluster.size();

            float base_area = (max_x-min_x)*(max_x-min_x) + (max_y-min_y)*(max_y-min_y);
            int distanceclus = pow(pow(centroid.x, 2)+pow(centroid.y, 2), 0.5)/ringlength;
            float angleclus = atan2(centroid.x, -centroid.y) - M_PI/6;
            int keyclus = int((angleclus)/(sectorangle)) + int((2*M_PI*distanceclus)/(3*sectorangle));

            if(keyclus%2==0){
              coeffsclus = plane[keyclus];
            }else{
              coeffsclus = plane[keyclus-1];
            }
            float denominatorclus = pow(pow(coeffsclus[0], 2) + pow(coeffsclus[1], 2) + pow(coeffsclus[2], 2), 0.5);
            float distclusfromground = abs((coeffsclus[0]*centroid.x + coeffsclus[1]*centroid.y + coeffsclus[2]*centroid.z + coeffsclus[3])/denominatorclus);


            if(distclusfromground <=0.4 && distclusfromground>=0.08
            // && max_z < 0.35-heightlidar 
            && base_area >=0.0 && base_area <=0.2
            ){
                std::cout << centroid.z+heightlidar << std::endl;
                centroids_cloud->points.push_back(centroid);
            }
            
            // ROS_INFO("Cluster %d centroid: [%f, %f, %f] with %lu points.", cluster_id++, centroid.x, centroid.y, centroid.z, cluster.size());
        }

        // Convert the centroids point cloud to ROS message
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*centroids_cloud, output);
        output.header = input->header;  // Keep the same header

        // Publish the centroids as a point cloud
        pub_.publish(output);
        ROS_INFO("Published %d centroids.", cluster_id);
  
}

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "luphy");
  ros::NodeHandle nh;
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI>> ("nogroundcloud", 1);
  pub_ = nh.advertise<sensor_msgs::PointCloud2>("/clusters", 1);
  marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
  ros::Subscriber sub = nh.subscribe ("rslidar_points", 1, cloud_cb);

  ros::spin ();
}






// #include "ros/ros.h"
// #include <sensor_msgs/PointCloud2.h>

// //plane removal
// #include <iostream>
// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/search/organized.h>

// //convert
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_types.h>
// #include <pcl/PCLPointCloud2.h>
// #include <pcl/conversions.h>
// #include <pcl_ros/transforms.h>

// //publish 
// #include <pcl_ros/point_cloud.h>
// // #include <pcl_ros/point_cloud.h>

// //remove Nan
// #include <pcl/filters/filter.h>
// #include "pcl-1.10/pcl/filters/impl/filter.hpp"
// #include <pcl/filters/passthrough.h>

// #define LidarHeight 0.265 //0.2
// #define Theta -0.024618149859381887 //0
// float cos_theta = cos(Theta);
// float sin_theta = sin(Theta);
// #define Threshold 0.01
// #define LidarSource "rslidar"
// #define LidarTopic "rslidar_points"
// #define MaxRadiusSq 35

// typedef pcl::PointXYZI PointType;

// class LidarHeightRemoval
// {
// public:
//     LidarHeightRemoval()
//     {
//         //LidarHeightRemovalHandle{};
//         // Ground = LidarHeightRemovalHandle.advertise<pcl::PointCloud<PointType>>("Ground", 10);
//         PointCloud = LidarHeightRemovalHandle.advertise<pcl::PointCloud<PointType>>("ConeCloud", 10);
//         Subscribe = LidarHeightRemovalHandle.subscribe(LidarTopic, 10, &LidarHeightRemoval::callback, this);
//         //timer(LidarHeightRemovalHandle.createTimer(ros::Duration(0.1), &LidarHeightRemoval::main_loop, this));
//     }

//     void callback(const sensor_msgs::PointCloud2ConstPtr& msg ) 
//     {
//         pcl::PCLPointCloud2 pcl_pc2;
//         pcl_conversions::toPCL(*msg,pcl_pc2);
//         pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
//         // pcl::PointCloud<PointType>::Ptr ground(new pcl::PointCloud<PointType>);
//         pcl::PointCloud<PointType>::Ptr cone_cloud(new pcl::PointCloud<PointType>);
//         // pcl::PointCloud<PointType>::Ptr ground_nan(new pcl::PointCloud<PointType>);
//         // pcl::PointCloud<PointType>::Ptr cone_cloud_nan(new pcl::PointCloud<PointType>);
//         // ground->header.frame_id = LidarSource;
//         // ground_nan->header.frame_id = LidarSource;
//         // ground->is_dense = "false";
//         cone_cloud->header.frame_id = LidarSource;
//         // cone_cloud_nan->header.frame_id = LidarSource;
//         cone_cloud->is_dense = "false";
//         pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
//         // float minimumx = INT_MAX;
//         // float minimumy = INT_MAX;
//         // float minimumz = INT_MAX;
//         // float maximumx = INT_MIN;
//         // float maximumy = INT_MIN;
//         // float maximumz = INT_MIN;
//         for (auto i : cloud->points) {
//             float rotated_x = i.x*cos_theta + i.z*sin_theta;
//             float rotated_z = -i.x*sin_theta + i.z*cos_theta;
//             // std::cout<<i.x<<" "<<i.y<<" "<<i.z<<std::endl;
//         //      if(i.z >maximumz){
//         //          maximumz = i.z;
//         //      }
//         //      if(i.y >maximumy){
//         //          maximumy = i.y;
//         //      }
//         //      if(i.x >maximumx){
//         //          maximumx = i.x;
//         //      }
//         //      if(i.x < minimumx){
//         //          minimumx = i.x;
//         //      }
//         //      if(i.y < minimumy){
//         //          minimumy = i.y;
//         //      }
//         //      if(i.z < minimumz){
//         //          minimumz = i.z;
//         //      }}
//         //    std::cout << minimumx <<" " <<maximumx <<" "<<minimumy <<" " <<maximumy <<" "<<minimumz <<" " <<maximumz <<" "<<std::endl;
//             if ((rotated_z + LidarHeight > Threshold)
//             // && (rotated_x*rotated_x + i.y*i.y < MaxRadiusSq)
//             && (abs(i.y) < 1.2) 
//             && (rotated_z + LidarHeight < 1.5)
//             && (i.x<13.0)
//         //   && (rotated_x*rotated_x + i.y*i.y > 7) 
//             ){
//                 cone_cloud->push_back(i);  
//             }
//             // else{
//             //     ground->push_back(i);
//             // }
//         }
//         //std::cout<<mini<<std::endl;
//         // std::vector<int> ind;
//         // pcl::removeNaNFromPointCloud(*ground, *ground, ind);
//         // pcl::PassThrough<PointType> pass;
//         // pass.setInputCloud (ground);
//         // pass.setFilterFieldName ("x");
//         // pass.setFilterLimits (-10, 10);
//         // //pass.setNegative (true);
//         // pass.filter (*ground);
//         // pass.setInputCloud (ground);
//         // pass.setFilterFieldName ("y");
//         // pass.setFilterLimits (-10, 10);
//         // //pass.setNegative (true);
//         // pass.filter (*ground);
//         // //pcl::removeNaNNormalsFromPointCloud(*ground, *ground, ind);
//         // ground->is_dense = "true";
//         // Ground.publish(*ground);        
//         // pcl::removeNaNFromPointCloud(*cone_cloud, *cone_cloud, ind);
//         // pass.setInputCloud (cone_cloud);
//         // pass.setFilterFieldName ("x");
//         // pass.setFilterLimits (-10, 10);
//         // //pass.setNegative (true);
//         // pass.filter (*cone_cloud);
//         // pass.setInputCloud (cone_cloud);
//         // pass.setFilterFieldName ("y");
//         // pass.setFilterLimits (-10, 10);
//         // //pass.setNegative (true);
//         // pass.filter (*cone_cloud);
//         // //pcl::removeNaNNormalsFromPointCloud(*cone_cloud, *cone_cloud, ind);
//         cone_cloud->is_dense = "true";
//         PointCloud.publish(*cone_cloud);
//         std::cout<<"Publishing Cone Cloud" <<std::endl;
//         //<<ground->size()<<" "<<cone_cloud->size()

//     }


// private:
//     ros::NodeHandle LidarHeightRemovalHandle;        // ROS node handle
//     ros::Publisher PointCloud;
//     // ros::Publisher Ground;        // ROS publisher for JointState messages
//     ros::Subscriber Subscribe;       // ROS subscriber for a topic
//     //ros::Timer timer;          // ROS timer for periodic tasks
// };

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "LidarHeightRemoval");
//     LidarHeightRemoval node;
//     ros::spin();
//     return 0;
// }