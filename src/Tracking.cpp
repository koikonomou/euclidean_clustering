#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <my_new_msgs/clustering.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/impl/point_types.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/point_cloud_conversion.h>

class Tracking{
private :
    ros::Publisher pub;
    ros::Subscriber sub;
public:

    int size, max_id;
    double overlap, offset;

    std::string out_topic;
    std::string input_topic;

    my_new_msgs::clustering c_;
    std::vector<my_new_msgs::clustering> v_;

    Tracking () {}

    void Centroid_tracker (const my_new_msgs::clustering& c_ ){
        sensor_msgs::PointCloud pc1;

        for (unsigned i=0; i < c_.clusters.size(); i++){
            pcl::PCLPointCloud2 cloud2;
            pcl_conversions::toPCL ( c_.clusters[i] , cloud2 );
            pcl::PointCloud<pcl::PointXYZ> cloud;
            pcl::fromPCLPointCloud2 ( cloud2 , cloud );
            pcl::CentroidPoint<pcl::PointXYZ> centroid;

            for (unsigned j=0; j < cloud.points.size(); j++){
                centroid.add(cloud.points[i]);
            }
            pcl::PointXYZ c1;
            centroid.get(c1);
        }
    }


    void init();

    void callback (const my_new_msgs::clustering& msg );

};

void Tracking::callback (const my_new_msgs::clustering& msg ){

    sensor_msgs::PointCloud cloud;
    my_new_msgs::clustering c_;
    v_.push_back(msg);

    if (v_.size() > size){
        v_.erase(v_.begin());
    }

    for (unsigned i=0; i < v_.size(); i++){
        double offset; 
        if ( i > 0 ){
            offset = ( 1.0 - overlap ) * (double)( ros::Duration( v_[i].first_stamp - v_[0].first_stamp ).toSec()) * (double)( msg.factor );
        }
        else{
            offset = 0.0;
        }

        for (unsigned j=0; j < v_[i].clusters.size(); j++){
            sensor_msgs::PointCloud cloud;
            sensor_msgs::convertPointCloud2ToPointCloud( v_[i].clusters[j] , cloud );

            for (unsigned k=0; k < cloud.points.size(); k++){
                cloud.points[k].z += - offset;
            }

            sensor_msgs::PointCloud2 pc2;
            sensor_msgs::convertPointCloudToPointCloud2( cloud , pc2 );

            c_.clusters.push_back( pc2 );
            c_.cluster_id.push_back( i );
            std::cout << " Cluster_id : " << c_.cluster_id.size() << " with " << cloud.points.size() << " data points "<< std::endl;
        }
    }
    pub.publish(c_);

}

void Tracking::init(){
    ros::NodeHandle n_;


    n_.param("Tracking/size", size , 2);
    n_.param("Tracking/overlap", overlap , 0.2);
    n_.param("Tracking/input_topic", input_topic , std::string("/new_pcl"));
    n_.param("Tracking/out_topic", out_topic , std::string("/Tracking"));


    sub = n_.subscribe( input_topic, 1 , &Tracking::callback, this);
    pub = n_.advertise<my_new_msgs::clustering>( out_topic, 1);

}


int main(int argc, char** argv){

    ros::init(argc, argv, "Tracking");

    Tracking tracker;
    tracker.init();

    ros::spin();

}