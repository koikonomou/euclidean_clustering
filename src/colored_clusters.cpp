#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/colors.h>
#include <pcl_ros/point_cloud.h>
#include <my_new_msgs/clustering.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/impl/point_types.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <pcl/common/projection_matrix.h>
#include <sensor_msgs/point_cloud_conversion.h>


ros::Publisher pub;
int counter;


void cluster_callback (const my_new_msgs::clustering& msg){

    sensor_msgs::PointCloud2 cluster_msgs;
    // pcl::PointCloud<pcl::PointXYZRGB> cloud_out;


    for (size_t i=0; i< msg.clusters.size(); i++){
        cluster_msgs.header.frame_id = "base_link";

        pcl::PCLPointCloud2 cloud2;
        pcl_conversions::toPCL( msg.clusters[i] , cloud2);

        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::fromPCLPointCloud2(cloud2, cloud);
        pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud = pcl::PointCloud<pcl::PointXYZRGB> (cloud);

        int j=0;

        for(size_t j=0; j < cloud.points.size(); j++){
            uint8_t r = 255 - 25 *i;
            uint8_t g = 90 + 40 *i;
            uint8_t b = 1024 * rand () / (RAND_MAX + 1.0f);
            int32_t rgb = (r << 16) | (g << 8) | b;
            cloud.points[j].rgb = *(float*)(&rgb);
        }

        pcl::PCLPointCloud2 clouds;
        pcl::toPCLPointCloud2(cloud, clouds);

        pcl_conversions::fromPCL(clouds, cluster_msgs);

        cluster_msgs.header.frame_id = "base_link";

        // sensor_msgs::PointCloud2 new_msg;
        // pcl::PCLPointCloud2 pcl2;
        // pcl::toPCLPointCloud2(cloud_out, pcl2);

        // pcl_conversions::fromPCL(pcl2, new_msg);

        // pcl::concatenatePointCloud(cluster_msgs, new_msg , cluster_msgs);
        pub.publish(cluster_msgs);
    }




}

int main (int argc, char** argv){
ros::init (argc, argv, "color_cluster");
ros::NodeHandle n_;

ros::Subscriber sub = n_.subscribe ("/new_pcl", 1, cluster_callback);

pub = n_.advertise<sensor_msgs::PointCloud2> ("rgb_cluster", 1);

ros::spin ();
}

