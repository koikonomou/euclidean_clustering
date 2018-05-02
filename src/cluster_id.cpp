#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <my_new_msgs/clustering.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

int size, overlap, offset;
ros::Publisher pub;
std::vector<my_new_msgs::clustering> v_;


void callback(const my_new_msgs::clustering& msg){

    sensor_msgs::PointCloud cloud;
    my_new_msgs::clustering c_;
    v_.push_back(msg);

    if (v_.size() > size){
        v_.erase(v_.begin());
    }

    for (unsigned i=0; i < v_.size(); i++){
        if ( i > 0 ){
            offset = overlap;
        }
        else{
            offset = 0 ;
        }

        for (unsigned j=0; j < v_[i].clusters.size(); j++){
            sensor_msgs::PointCloud cloud;
            sensor_msgs::convertPointCloud2ToPointCloud( v_[i].clusters[j] , cloud);

            for (unsigned k=0; k < cloud.points.size(); k++){
                cloud.points[k].z += - offset;
            }

            sensor_msgs::PointCloud2 pc2;
            sensor_msgs::convertPointCloudToPointCloud2( cloud , pc2 );

            c_.clusters.push_back(pc2);
            c_.cluster_id.push_back(i) ;
            std::cout << " Cluster_id : " << c_.cluster_id.size() << " with " << cloud.points.size() << " data points "<< std::endl;
        }

    }
    pub.publish(c_);

}



int main ( int argc, char** argv){
    ros::init (argc, argv, "cluster_id");
    ros::NodeHandle n_;
    n_.param("cluster_id/size", size , 2);
    n_.param("cluster_id/overlap", overlap , 2);

    ros::Subscriber sub = n_.subscribe("/new_pcl", 1 , callback);
    pub = n_.advertise<my_new_msgs::clustering>("cluster_id", 1);
    ros::spin ();


}