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


void cluster_callback (const my_new_msgs::clustering& msg){

    sensor_msgs::PointCloud2 accumulator;

    sensor_msgs::PointCloud2 cluster_msgs;

    for (size_t i=0; i < msg.clusters.size(); i++){

        pcl::PCLPointCloud2 cloud2;
        pcl_conversions::toPCL( msg.clusters[i] , cloud2);

        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::fromPCLPointCloud2(cloud2, cloud);

        if (msg.cluster_id.size() > 0 ){
            for (int u=0; u < msg.cluster_id.size(); u++){
                ROS_WARN("%u", u);

                for(size_t j=0; j < cloud.points.size(); j++){
                    uint8_t r = 255 - 25 * u^8 *i;
                    uint8_t g = 90 + 40 * u^8 *i;
                    uint8_t b = 40; 
                    int32_t rgb = (r << 16) | (g << 8) | b;
                    cloud.points[j].rgb = *(float*)(&rgb);
                }
            }
        }
        else {
            for(size_t j=0; j < cloud.points.size(); j++){
                uint8_t r = 255 - 25 * i;
                uint8_t g = 90 + 40 * i;
                uint8_t b = 40; 
                int32_t rgb = (r << 16) | (g << 8) | b;
                cloud.points[j].rgb = *(float*)(&rgb);
            }
        }
        pcl::PCLPointCloud2 clouds;
        pcl::toPCLPointCloud2(cloud, clouds);
        pcl_conversions::fromPCL(clouds, cluster_msgs);
        cluster_msgs.header.frame_id = "base_link";

        sensor_msgs::PointCloud2 tmp = sensor_msgs::PointCloud2(accumulator);

        pcl::concatenatePointCloud( cluster_msgs, tmp, accumulator);
    }

    pub.publish(accumulator);
}

int main (int argc, char** argv){
    ros::init (argc, argv, "colored_cluster");
    ros::NodeHandle n_;
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);


    std::string input_topic;
    std::string out_topic;
    n_.param("color_clustered/cloud_topic",input_topic, std::string("/new_pcl"));
    n_.param("color_clustered/output_cloud_topic", out_topic, std::string("/rgb_cluster"));

    ros::Subscriber sub = n_.subscribe (input_topic, 1, cluster_callback);

    pub = n_.advertise<sensor_msgs::PointCloud2> (out_topic, 1);

    ros::spin ();
}

