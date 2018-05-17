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

    int max_id ; 
    double max_dist ;

    my_new_msgs::clustering base_msg;

    Centroid_tracking ( my_new_msgs::clustering& base_msg, int max_id ) 
    {
        this->base_msg = base_msg ;
        this->max_id = max_id ;
        this->max_dist = 0;
    }

    void track ( my_new_msgs::clustering& msg ) {

        std::vector<int> base_id;
        std::vector<int> auth_msg_id; //authentication id 

        std::vector<Eigen::Vector4f> msg_centroid_vec;
        std::vector<Eigen::Vector4f> base_centroid_vec;

        //first frame 
        for (int i=0; i < base_msg.clusters.size(); i++)
        {
            pcl::PointXYZ centroidpoint ;
            pcl::PCLPointCloud2 pc2;
            pcl_conversions::toPCL ( base_msg.clusters[i] , pc2 );

            pcl::PointCloud<pcl::PointXYZ> cloud2;
            pcl::fromPCLPointCloud2 ( pc2 , cloud2 );

            Eigen::Vector4f base_centroid;
            pcl::compute3DCentroid ( cloud2 , base_centroid);
            std::cerr << " Cluster_id " << base_msg.cluster_id[i] << "  centroid_x : " << base_centroid(0) << " centroid_y : " << base_centroid(1) << " centroid_z : " << base_centroid(2) << std::endl;

            base_centroid_vec.push_back( base_centroid );

            base_id.push_back( base_msg.cluster_id[i] );
        }
        //second frame
        for (int i=0; i < msg.clusters.size(); i++)
        {
            pcl::PointXYZ centroidpoint ;
            pcl::PCLPointCloud2 pc2;
            pcl_conversions::toPCL ( msg.clusters[i] , pc2 );

            pcl::PointCloud<pcl::PointXYZ> cloud2;
            pcl::fromPCLPointCloud2 ( pc2 , cloud2 );
            // ROS_INFO("%lu" , cloud2.points.size());

            Eigen::Vector4f base_centroid;
            pcl::compute3DCentroid ( cloud2 , base_centroid);
            std::cerr << " ---Cluster_id " << msg.cluster_id[i] << "  centroid_x : " << base_centroid(0) << " centroid_y : " << base_centroid(1) << " centroid_z : " << base_centroid(2) << std::endl;

            msg_centroid_vec.push_back( base_centroid );
            auth_msg_id.push_back( msg.cluster_id[i]);

        }

        //overwrite new msg_id to check the clusters that I used 
        double min_value = std::numeric_limits<double>::min() ;

        for (int i=0; i < auth_msg_id.size(); i++){
            auth_msg_id[i] = ( int ) min_value ;
        }


        for (int i=0; i < base_centroid_vec.size(); i++)
        {
            Eigen::Vector4f dist;
            double dist_x , dist_y , dist_z;

            int cnt = 0 ;
            int min_index = -1;
            int b_min_index = -1 ;

            double min_dist = std::numeric_limits<double>::max() ;

            for (int j=0; j < msg_centroid_vec.size(); j++)
            {
                dist = base_centroid_vec[i] - msg_centroid_vec[j];
                dist_x = dist(0) ;
                dist_y = dist(1) ;
                dist_z = dist(2) ;

                //compute distance between centroids
                double real_dist ;
                real_dist = sqrt( pow( dist_x , 2 ) + pow( dist_y, 2 ) + pow( dist_z , 2 ) );

                std::vector<double> dist_vec ;
                dist_vec.push_back( real_dist );

                if ( dist_vec[j] < min_dist && auth_msg_id[j] == ( int ) min_value ){
                    min_dist = dist_vec[j] ;
                    min_index = j ;
                    auth_msg_id[j] = min_index ;
                }
                else {
                    // cnt++ ;
                }
            }

            if ( min_dist < max_dist ){
                msg.cluster_id[ min_index ] = base_id[i] ;
            }
            else {
                msg.cluster_id[ min_index ] = max_id + 1;
            }

            for ( int i=0 ; i < auth_msg_id.size(); i++){
                if ( auth_msg_id[i] == (( int ) min_value) ){
                    cnt++ ;
                    b_min_index = i;
                    auth_msg_id[i] = b_min_index;
                    msg.cluster_id[ b_min_index ] = max_id + cnt ;
                }
            }

        }

    }

};

ros::Publisher pub;
ros::Subscriber sub;

bool b = true;
int size , max_id ;
double overlap, offset , max ;

std::vector<my_new_msgs::clustering> v_;

void callback (const my_new_msgs::clustering& msg ){

    my_new_msgs::clustering c_;
    sensor_msgs::PointCloud cloud;

    v_.push_back(msg);

    Centroid_tracking* t;

    if (v_.size() > size){
        v_.erase(v_.begin());
        if ( b ){
            for (unsigned i=0 ; i < v_[0].clusters.size(); i++){
                v_[0].cluster_id.push_back(i);
            }
            b = false;
        }
        for (int i=0 ; i < v_[1].clusters.size(); i++){
            v_[1].cluster_id.push_back(i);
        }

        for (unsigned i=0; i < v_[0].cluster_id.size(); i++){
            if (v_[0].cluster_id[i] > max_id){
                max_id = v_[0].cluster_id[i];
            }
        }

        t = new Centroid_tracking( v_[0] , max_id ) ;
    }

    else {
        t = NULL;
    }

    if ( t !=NULL ) {
        t->max_dist = max;
        t->track( v_[1] );
    }

    for (unsigned i=0; i < v_.size(); i++)
    {
        double offset;

        if ( i > 0 ){
            offset = ( 1.0 - overlap ) * (double)( ros::Duration( v_[i].first_stamp - v_[0].first_stamp ).toSec()) * (double)( msg.factor );
        }
        else {
            offset = 0.0;
        }

        for (unsigned j=0; j < v_[i].clusters.size(); j++){
            sensor_msgs::PointCloud cloud;
            sensor_msgs::convertPointCloud2ToPointCloud( v_[i].clusters[j] , cloud );

            for (unsigned k=0; k < cloud.points.size(); k++){
                cloud.points[k].z += offset;
            }

            sensor_msgs::PointCloud2 pc2;
            sensor_msgs::convertPointCloudToPointCloud2( cloud , pc2 );
            c_.clusters.push_back( pc2 );
            for (int i=0; i < v_[0].cluster_id.size(); i++){
                c_.cluster_id.push_back(v_[0].cluster_id[i]);
            }

        }
    }

    pub.publish(c_);

}


int main(int argc, char** argv){

    ros::init(argc, argv, "Tracking");
    ros::NodeHandle n_;

    std::string out_topic;
    std::string input_topic;


    n_.param("Tracking/size", size , 2);
    n_.param("Tracking/max_dist", max , 9.0);
    n_.param("Tracking/overlap", overlap , 0.2);

    n_.param("Tracking/out_topic", out_topic , std::string("/Tracking"));
    n_.param("Tracking/input_topic", input_topic , std::string("/new_pcl"));

    sub = n_.subscribe( input_topic, 1 , callback);
    pub = n_.advertise<my_new_msgs::clustering>( out_topic, 1);

    ros::spin();
}