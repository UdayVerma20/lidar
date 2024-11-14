#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <cstdlib>  // For rand()

#define heightlidar 0.28

class DBSCAN
{
public:
    DBSCAN(double eps, int minPts) : eps_(eps), minPts_(minPts) {}

    void cluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<std::vector<int>>& clusters)
    {
        std::vector<bool> visited(cloud->points.size(), false);
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
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
    bool expandCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree,
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

class ClusterExtractionNode
{
public:
    ClusterExtractionNode(ros::NodeHandle& nh) :
        nh_(nh)
    {
        sub_ = nh_.subscribe("/nogroundcloud", 1, &ClusterExtractionNode::pointCloudCallback, this);
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/clusters", 1);  // Change topic to indicate centroids
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        // ROS_INFO("Received a point cloud");

        // Convert the ROS PointCloud2 message to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        // ROS_INFO("Converted to PCL PointCloud, number of points: %zu", cloud->points.size());

        // Optionally downsample the cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(cloud);
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
            
            if(centroid.z<=0.3-heightlidar //&& centroid.z>=-heightlidar 
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
        output.header = cloud_msg->header;  // Keep the same header

        // Publish the centroids as a point cloud
        pub_.publish(output);
        ROS_INFO("Published %d centroids.", cluster_id);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dbscan");
    ros::NodeHandle nh;

    ClusterExtractionNode node(nh);

    ros::spin();
    return 0;
}
