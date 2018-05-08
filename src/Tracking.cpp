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

class Tracker{
private:
    my_new_msgs::clustering base_msg;
    int max_id;
public:

    int size, id;
    double overlap, offset ;
    double threshold_x , threshold_y , threshold_z ;

    ros::Publisher pub;
    ros::Subscriber sub;

    std::string out_topic;
    std::string input_topic;


    Tracker (my_new_msgs::clustering base_msg, int max_id ) : base_msg(base_msg) , max_id(max_id) { }

    void track (const my_new_msgs::clustering msg ) {
        std::vector<Eigen::Vector4f> base_centroid_vec;
        std::vector<Eigen::Vector4f> msg_centroid_vec;
        std::vector<int> base_id;

        for (int i = 0; i < base_msg.clusters.size(); i++)
        {
            pcl::PointXYZ centroidpoint ;
            pcl::PCLPointCloud2 pc2;
            pcl_conversions::toPCL ( base_msg.clusters[i] , pc2 );
            pcl::PointCloud<pcl::PointXYZ> cloud2;
            pcl::fromPCLPointCloud2 ( pc2 , cloud2 );
            Eigen::Vector4f base_centroid;
            pcl::compute3DCentroid ( cloud2 , base_centroid);

            base_centroid_vec.push_back(base_centroid);
            base_id.push_back(base_msg.cluster_id[i]);
        }

        for (int i = 0; i < msg.clusters.size(); i++)
        {
            pcl::PointXYZ centroidpoint ;
            pcl::PCLPointCloud2 pc2;
            pcl_conversions::toPCL ( msg.clusters[i] , pc2 );
            pcl::PointCloud<pcl::PointXYZ> cloud2;
            pcl::fromPCLPointCloud2 ( pc2 , cloud2 );
            Eigen::Vector4f base_centroid;
            pcl::compute3DCentroid ( cloud2 , base_centroid);
            msg_centroid_vec.push_back(base_centroid);
        }

        for (int i = 0; i < base_centroid_vec.size(); i++){
            std::vector<int> dist;
            for (int j=0; j < msg_centroid_vec.size(); j++){
/*              dist vector msg[i]-msg[j]
                k = min_dist
                msg[k].id = base_id[i]*/

            }

        }


    }


    void init();

    void callback (const my_new_msgs::clustering& msg );

};

void Tracker::callback (const my_new_msgs::clustering& msg ){

    int cnt = 0;
    int max_id = 0;

    my_new_msgs::clustering c_;
    sensor_msgs::PointCloud cloud;
    std::vector<my_new_msgs::clustering> v_;

    v_.push_back(msg);

    if (v_.size() > size){
        v_.erase(v_.begin());
    }
    Tracker* t = new Tracker( v_[0] , max_id ) ;

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
            c_.cluster_id.push_back( cnt );

            std::cout << " Cluster_id : " << c_.cluster_id.size() << " with " << cloud.points.size() << " data points "<< std::endl;
        }
        cnt++;

    }

    pub.publish(c_);

}

void Tracker::init(){
    ros::NodeHandle n_;

    n_.param("Tracking/size", size , 2);
    n_.param("Tracking/overlap", overlap , 0.2);
    n_.param("Tracking/threshold_x", threshold_x , 0.5);
    n_.param("Tracking/threshold_y", threshold_y , 0.5);
    n_.param("Tracking/threshold_z", threshold_z , 0.5);
    n_.param("Tracking/out_topic", out_topic , std::string("/Tracking"));
    n_.param("Tracking/input_topic", input_topic , std::string("/new_pcl"));


    sub = n_.subscribe( input_topic, 1 , &Tracker::callback, this);
    pub = n_.advertise<my_new_msgs::clustering>( out_topic, 1);

}


int main(int argc, char** argv){

    ros::init(argc, argv, "Tracking");

    // t->init();
    ros::spin();

}