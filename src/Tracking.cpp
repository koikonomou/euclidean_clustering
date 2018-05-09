#include <cmath>
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

class Centroid_tracking{
public:

    int size , max_id;
    double overlap, offset ;

    ros::Publisher pub;
    ros::Subscriber sub;

    std::string out_topic;
    std::string input_topic;

    my_new_msgs::clustering base_msg;
    std::vector<my_new_msgs::clustering> v_;

    Centroid_tracking (const my_new_msgs::clustering base_msg, int max_id ) { }

    void track ( my_new_msgs::clustering& msg ) {

        std::vector<int> base_id;
        std::vector<Eigen::Vector4f> msg_centroid_vec;
        std::vector<Eigen::Vector4f> base_centroid_vec;

        for (int i=0; i < base_msg.clusters.size(); i++)
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

        for (int i=0; i < msg.clusters.size(); i++)
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

        for (int i=0; i < base_centroid_vec.size(); i++)
        {
            Eigen::Vector4f dist;
            int dist_x , dist_y , dist_z;

            for (int j=0; j < msg_centroid_vec.size(); j++)
            {
                dist = base_centroid_vec[i] - msg_centroid_vec[j];
                dist_x = dist(0) ;
                dist_y = dist(1) ;
                dist_z = dist(2) ;

                double real_dist ;
                real_dist = sqrt( pow( dist_x , 2 ) + pow( dist_y, 2 ) + pow( dist_z , 2 ) );

                std::vector<double> dist_vec;
                dist_vec.push_back( real_dist );

                double min_dist;

                if ( j > 0 ){
                    if ( dist_vec[j] < min_dist ){
                        min_dist = dist_vec[j];
                        msg.cluster_id[j] = base_id[i] ;
                    }
                }
                else {
                    min_dist = dist_vec[0];
                }
            }

        }


    }

    void init();

    void callback (const my_new_msgs::clustering& msg );

};

void Centroid_tracking::callback (const my_new_msgs::clustering& msg ){

    my_new_msgs::clustering c_;
    sensor_msgs::PointCloud cloud;

    v_.push_back(msg);

    if (v_.size() > size){
        v_.erase(v_.begin());
    }

    Centroid_tracking* t = new Centroid_tracking( v_[0] , max_id ) ;

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

void Centroid_tracking::init(){
    ros::NodeHandle n_;

    n_.param("Tracking/size", size , 2);
    n_.param("Tracking/overlap", overlap , 0.2);

    n_.param("Tracking/out_topic", out_topic , std::string("/Tracking"));
    n_.param("Tracking/input_topic", input_topic , std::string("/new_pcl"));


    sub = n_.subscribe( input_topic, 1 , &Centroid_tracking::callback, this);
    pub = n_.advertise<my_new_msgs::clustering>( out_topic, 1);

}


int main(int argc, char** argv){

    ros::init(argc, argv, "Tracking");

    int max_id ;
    my_new_msgs::clustering c;

    Centroid_tracking t( c, max_id );

    t.init();
    ros::spin();

}