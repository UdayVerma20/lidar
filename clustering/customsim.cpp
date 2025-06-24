#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/filter.h>
#include <clustering/CoordinateList.h>
#include <clustering/Coordinates.h>
#include <ground_removal/arr.h>
#include <ground_removal/arrofarr.h>
#include<bits/stdc++.h>
#include <pcl_ros/point_cloud.h>
using namespace std;
#define heightlidar 0.188
#define lidarframe "velodyne"
#define sectorangle M_PI/48
#define planespreadbeforering 70
#define ringlength 0.2
#define xthresh 3
//ConeCheck
#define MaxHeight 0.4
#define MinHeight 0.05
// #define MinHeight 0.1 //0.2
// #define MaxPoints 2000
#define MinPoints 1
// #define MaxLen 1 //0.7
#define MaxWidth 0.6
#define MaxLen 0.5

//thresholds
float EuclideanThreshold =  0.1;
float CentroidThreshold = 0.3;

unordered_map<int, vector<float>> ground;
pcl::PointXYZI lidarpoint;

class CustomCluster{
    public:
    int clustersize = 0;
    pcl::PointXYZI Avg;
    pcl::PointXYZI Last;
    pcl::PointXYZI Left;
    pcl::PointXYZI Right;
    pcl::PointXYZI Front;
    pcl::PointXYZI Back;
    pcl::PointXYZI minheight;
    pcl::PointXYZI maxheight;

    CustomCluster(pcl::PointXYZI Point){
        Avg = Point;
        Last = Point;
        Left = Point;
        Right = Point;
        Front = Point;
        Back = Point;
        clustersize = 1;
        minheight = maxheight = Point;
    }
};

int check_distance(pcl::PointXYZI Cluster_Point, pcl::PointXYZI point, float Threshold){
    float distance = pow(((float)Cluster_Point.x  - (float)point.x), 2.0) + pow(((float)Cluster_Point.y  - (float)point.y), 2.0)
     + pow(((float)Cluster_Point.z  - (float)point.z), 2.0);
    return (distance < Threshold); 
}

class Clustering
{
public:
    Clustering(ros::NodeHandle ClusteringHandle){
        lidarpoint.x = 0.f; lidarpoint.y = 0.f; lidarpoint.z = -heightlidar;
        Clusters = ClusteringHandle.advertise<clustering::CoordinateList>("Clusters", 1000);
        Clusters_pc = ClusteringHandle.advertise<pcl::PointCloud<pcl::PointXYZI>>("Clusters_PointCloud", 1000);
        Surrounding_pc = ClusteringHandle.advertise<pcl::PointCloud<pcl::PointXYZI>>("Surrounding_PointCloud", 1000);
        subground = ClusteringHandle.subscribe ("ground", 1, &Clustering::callbackground, this);
        sub = ClusteringHandle.subscribe ("ConeCloud", 1, &Clustering::callback, this);
    }

    void callbackground(ground_removal::arrofarr inputplanes){
        ground.clear();
        for(auto i:inputplanes.data){
            ground[int(round(i.data[0]))] = {i.data[1], i.data[2], i.data[3], i.data[4]};
        }
    }
    void callback(const sensor_msgs::PointCloud2ConstPtr& input){   
        // if (ground.size()==0) return;
        // if(line.size()!=0){ 
        clustering::CoordinateList Cluster;
        Cluster.header.stamp = ros::Time::now();
        Cluster.header.frame_id = lidarframe;
        Cluster.size = 0;
        std::vector<CustomCluster> Clusters_Vector;
        CustomCluster StdCluster(pcl::PointXYZI(0.f));
        Clusters_Vector.push_back(StdCluster);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg (*input, *cloud);

        if (cloud->size()!= 0){
            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_pc (new pcl::PointCloud<pcl::PointXYZI>);
            cluster_pc->header.frame_id = lidarframe;
            pcl::PointCloud<pcl::PointXYZI>::Ptr surr_cloud (new pcl::PointCloud<pcl::PointXYZI>);
            surr_cloud->header.frame_id = lidarframe;

            for (pcl::PointXYZI point : cloud->points){
                if (isfinite(point.x) && isfinite(point.y) && isfinite(point.z)){
                    // if ((point.x > xthresh) || (point.x < 1)) continue;
                    int found_cluster = 0;
                    for (int index = 0; index < Clusters_Vector.size(); index++){
                        CustomCluster Iter_Cluster = Clusters_Vector[index];
                        if (
                            check_distance(Clusters_Vector[index].Last, point, EuclideanThreshold) ||
                            check_distance(Clusters_Vector[index].Avg, point, CentroidThreshold) ||
                            check_distance(Clusters_Vector[index].Left, point, EuclideanThreshold) ||
                            check_distance(Clusters_Vector[index].Right, point, EuclideanThreshold) ||
                            check_distance(Clusters_Vector[index].minheight, point, EuclideanThreshold) ||
                            check_distance(Clusters_Vector[index].maxheight, point, EuclideanThreshold) ||
                            check_distance(Clusters_Vector[index].Front, point, EuclideanThreshold) ||
                            check_distance(Clusters_Vector[index].Back, point, EuclideanThreshold)                        
                            ){

                            //after increment
                            int current_size = (Clusters_Vector[index].clustersize++);
                            double inv_current_size = (double)1/(current_size+1);

                            Clusters_Vector[index].Last = point;
                            Clusters_Vector[index].Left =  (Iter_Cluster.Left.y < point.y) ? point: Iter_Cluster.Left ;
                            Clusters_Vector[index].Right = (Iter_Cluster.Right.y > point.y) ? point: Iter_Cluster.Right ;
                            Clusters_Vector[index].Front =  (Iter_Cluster.Front.x > point.x) ? point: Iter_Cluster.Front ;
                            Clusters_Vector[index].Back = (Iter_Cluster.Back.x < point.x) ? point: Iter_Cluster.Back ;
                            Clusters_Vector[index].minheight =  (Iter_Cluster.minheight.z > point.z) ? point: Iter_Cluster.minheight ;
                            Clusters_Vector[index].maxheight = (Iter_Cluster.maxheight.z < point.z) ? point: Iter_Cluster.maxheight ;

                            Clusters_Vector[index].Avg.x = (double)(1 - inv_current_size)*(double)Iter_Cluster.Avg.x + (double)(point.x*inv_current_size) ;
                            Clusters_Vector[index].Avg.y = (double)(1 - inv_current_size)*(double)Iter_Cluster.Avg.y + (double)(point.y*inv_current_size) ;
                            Clusters_Vector[index].Avg.z = //(Iter_Cluster.Avg.z < point.z) ? Iter_Cluster.Avg.z : point.z;
                            (double)(1 - inv_current_size)*(double)Iter_Cluster.Avg.z + (double)(point.z*inv_current_size) ;
                            
                            found_cluster++;

                            // if (Iter_Cluster.minheight.z > point.z){
                            //     Clusters_Vector[index].minheight = point;
                            // }
                            // if (Iter_Cluster.maxheight.z < point.z){
                            //     Clusters_Vector[index].maxheight = point;
                            // }

                            surr_cloud->push_back(point);
                            break;
                        }
                    }
                    if (!found_cluster){
                        CustomCluster NewCluster(point);
                        Clusters_Vector.push_back(NewCluster);
                    }
                }
            }

            Surrounding_pc.publish(*surr_cloud);

            cluster_pc->width = Clusters_Vector.size();
            cluster_pc->height = 1;
            // cluster_pc->push_back(pcl::PointXYZI(0.f));
            float denominator = 1;
            std::cout<<Clusters_Vector.size()<<std::endl;
            for (int index = 0; index < Clusters_Vector.size(); index++){
                CustomCluster Iter_Cluster = Clusters_Vector[index];
                if (Iter_Cluster.Avg.x == 0.0 && Iter_Cluster.Avg.y == 0.0 && Iter_Cluster.Avg.z == 0.0 ) continue;
                int dist_sq = pow(Iter_Cluster.Avg.x, 2.0) + pow(Iter_Cluster.Avg.y, 2.0) + pow(Iter_Cluster.Avg.z, 2.0);
                int expected_points = (2000)/((float)(dist_sq + 1)); //remove dist =0
                if (!(Iter_Cluster.Avg.x == 0 && Iter_Cluster.Avg.y ==0)){
                    float ratio = Iter_Cluster.Avg.y/abs(Iter_Cluster.Avg.x);
                    if ((ratio > tan(10) && ratio < tan(15)) || (ratio > tan(30) && ratio < tan(40))){
                        expected_points *= 2;
                    }
                }
                
                //checking cone distance from lu planes
                // int ringclus = pow(pow(Iter_Cluster.Avg.x, 2)+pow(Iter_Cluster.Avg.y, 2), 0.5)/ringlength;
                // float angleclus = atan2(Iter_Cluster.Avg.x, -Iter_Cluster.Avg.y) - M_PI/6;
                // int key = int((angleclus)/(sectorangle)) + int((2*M_PI*ringclus)/(3*sectorangle));
                
                // denominator = pow(pow(ground[key][0], 2) + pow(ground[key][1], 2) + pow(ground[key][2], 2), 0.5);
                // float distance = abs(Iter_Cluster.Avg.x*ground[key][0] + Iter_Cluster.Avg.y*ground[key][1] + Iter_Cluster.Avg.z*ground[key][2] + ground[key][3])/denominator;
                // float minzfromground = abs(Iter_Cluster.minheight.x*ground[key][0] + Iter_Cluster.minheight.y*ground[key][1] + Iter_Cluster.minheight.z*ground[key][2] + ground[key][3])/denominator;
                // float maxzfromground = abs(Iter_Cluster.maxheight.x*ground[key][0] + Iter_Cluster.maxheight.y*ground[key][1] + Iter_Cluster.maxheight.z*ground[key][2] + ground[key][3])/denominator;


                if ( //checks
                1
                // && (distance >=0.04 && distance < 0.32 )
                // && (Iter_Cluster.Avg.z + heightlidar < 1.5)
                // // && (Iter_Cluster.Avg.z + heightlidar < 0.2)
                // // && (Iter_Cluster.Avg.z + heightlidar > MinHeight)
                // && (maxzfromground >= 0.1)
                // && (minzfromground <= 0.22 )
                // && (Iter_Cluster.clustersize < 1000)
                // // && (Iter_Cluster.clustersize > 0.2 * expected_points)
                // && ((Iter_Cluster.Right.y - Iter_Cluster.Left.y)*(Iter_Cluster.Right.y - Iter_Cluster.Left.y) + (Iter_Cluster.Right.x - Iter_Cluster.Left.x)*(Iter_Cluster.Right.x - Iter_Cluster.Left.x) < MaxWidth*MaxWidth)
                // && ((Iter_Cluster.Back.y - Iter_Cluster.Front.y)*(Iter_Cluster.Back.y - Iter_Cluster.Front.y) + (Iter_Cluster.Back.x - Iter_Cluster.Front.x)*(Iter_Cluster.Back.x - Iter_Cluster.Front.x) < MaxLen*MaxLen)
                && (Iter_Cluster.clustersize > MinPoints)
                ){
                    // cout<<expected_points<<" "<<Iter_Cluster.clustersize<<endl;
                    Cluster.size++;
                    clustering::Coordinates CurrentCluster;
                    CurrentCluster.size = Iter_Cluster.clustersize;
                    CurrentCluster.reconsize = 0;
                    CurrentCluster.x = Iter_Cluster.Avg.x;
                    CurrentCluster.y = Iter_Cluster.Avg.y;
                    CurrentCluster.z = Iter_Cluster.Avg.z;
                    CurrentCluster.left = {Iter_Cluster.Left.x, Iter_Cluster.Left.y, Iter_Cluster.Left.z};
                    CurrentCluster.right = {Iter_Cluster.Right.x, Iter_Cluster.Right.y, Iter_Cluster.Right.z};
                    CurrentCluster.front = {Iter_Cluster.Front.x, Iter_Cluster.Front.y, Iter_Cluster.Front.z};
                    CurrentCluster.back = {Iter_Cluster.Back.x, Iter_Cluster.Back.y, Iter_Cluster.Back.z};
                    CurrentCluster.top = {Iter_Cluster.maxheight.x, Iter_Cluster.maxheight.y, Iter_Cluster.maxheight.z};
                    CurrentCluster.bottom = {Iter_Cluster.minheight.x, Iter_Cluster.minheight.y, Iter_Cluster.minheight.z};
                    std::cout << "Cone " 
                    <<Iter_Cluster.clustersize<<" " <<Iter_Cluster.Avg.x<<" "<<Iter_Cluster.Avg.y<<" "<<Iter_Cluster.Avg.z
                    // <<" "<<distance <<" "<<minzfromground<<" "<<maxzfromground
                    <<std::endl; 
                    Cluster.ConeCoordinates.push_back(CurrentCluster);
                    cluster_pc->push_back(Iter_Cluster.Avg);
                
                }
            }
            cluster_pc->push_back(pcl::PointXYZI(0.f));
            Clusters_pc.publish(*cluster_pc);
        }
        else{
            std::cout<<"No object found"<<std::endl;
        }
        Clusters.publish(Cluster);
        std::cout<<"Publishing Cone Coordinates"<<std::endl;
    }

private:
    ros::Publisher Clusters;
    ros::Publisher Clusters_pc;
    ros::Publisher Surrounding_pc;
    ros::Subscriber sub;
    ros::Subscriber subground;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "customsim");
    ros::NodeHandle ClusteringHandle;
    Clustering node(ClusteringHandle);
    ros::spin();
    return 0;
}
