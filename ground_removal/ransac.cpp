#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
// #include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
// #include <pcl/conversions.h>
// #include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#define Xrange 5.0
#define Yrange 1.0

int maxIterations = 500;
float distanceThreshold = 0.1;

pcl::PointCloud<pcl::PointXYZI>::Ptr cone_cloud(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);

// Function to calculate the distance of a point to a plane
float pointToPlaneDistance(const pcl::PointXYZI p, float a, float b, float c, float d) {
    return fabs(a * p.x + b * p.y + c * p.z + d) / sqrt(a * a + b * b + c * c);
}

bool pointinrange(pcl::PointXYZI Point){
    return (abs(Point.y) < Yrange && Point.x < Xrange);
}

class Ransac
{
public:
    Ransac()
    {
        //RansacHandle{};
        ConeCloudPc = RansacHandle.advertise<pcl::PointCloud<pcl::PointXYZI>>("ConeCloud", 1000);
        GroundPc = RansacHandle.advertise<pcl::PointCloud<pcl::PointXYZI>>("GroundCloud", 1000);
        // All_Clusters_pc = RansacHandle.advertise<pcl::PointCloud<PointType>>("All_Clusters_PointCloud", 100);
        Subscribe = RansacHandle.subscribe("rslidar_points", 10, &Ransac::callback, this);
        //timer(RansacHandle.createTimer(ros::Duration(0.1), &GroundRemoval::main_loop, this));
    }

    // Function to implement RANSAC to find the best-fitting plane
    void callback(const pcl::PointCloud<pcl::PointXYZI>::Ptr& point_cloud) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr ransac_cone_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ransac_ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        int numPoints = point_cloud->size();
        srand(time(0)); // Seed random number generator

        int bestInlierssize= 0;
        int i = 0;
        while (i < maxIterations) {
            i++;
            cone_cloud->clear();
            ground_cloud->clear();

            // Randomly select 3 points to define a plane
            int idx1 = rand() % numPoints;
            int idx2 = rand() % numPoints;
            int idx3 = rand() % numPoints;

            // Ensure points are unique and in range
            if (idx1 == idx2 || idx2 == idx3 || idx1 == idx3) continue;

            pcl::PointXYZI p1 = point_cloud->points[idx1];
            pcl::PointXYZI p2 = point_cloud->points[idx2];
            pcl::PointXYZI p3 = point_cloud->points[idx3];

            if (pointinrange(p1) || pointinrange(p2) || pointinrange(p3)) continue;

            // Compute the plane parameters (a, b, c, d) from the points
            float a = (p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y);
            float b = (p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z);
            float c = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
            float d = -(a * p1.x + b * p1.y + c * p1.z);

            // Count inliers
            // std::vector<int> inliers;
            int Inlierssize = 0;
            for (auto point : point_cloud->points) {
                float curr_dist = pointToPlaneDistance(point, a, b, c, d);
                if (curr_dist < distanceThreshold) {
                    pcl::PointXYZI p(0.f);
                    p.x = -point.x;
                    p.y = point.y;
                    p.z = point.z;
                    ground_cloud->push_back(p);
                    Inlierssize++;
                }
                else{
                    cone_cloud->push_back(point);
                    
                }
            }

            // Update the best plane if we found a better model
            if (Inlierssize > bestInlierssize) {
                ransac_cone_cloud = cone_cloud;
                ransac_ground_cloud = ground_cloud;
                bestInlierssize= Inlierssize;
            }
            // i++;
        }
        ransac_cone_cloud->header.frame_id = "rslidar";
        ransac_ground_cloud->header.frame_id = "rslidar";
        ConeCloudPc.publish(*ransac_cone_cloud);
        GroundPc.publish(*ransac_ground_cloud);
        std::cout<<"Publishing ConeCloud and GroundCloud"<<std::endl;
    }

private:
    ros::NodeHandle RansacHandle;        // ROS node handle
    ros::Publisher ConeCloudPc;       // ROS publisher for JointState messages
    ros::Publisher GroundPc;
    // ros::Publisher All_Clusters_pc;
    ros::Subscriber Subscribe;       // ROS subscriber for a topic
    //ros::Timer timer;          // ROS timer for periodic tasks
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Ransac");
    Ransac node;
    ros::spin();
    // srand(time(0)); 
    // while(1){
    //     std::cout<<rand()<<std::endl;
    // }
    return 0;
}

