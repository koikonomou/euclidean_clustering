#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <my_new_msgs/clustering.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/impl/point_types.hpp>
#include <sensor_msgs/ChannelFloat32.h>
#include <pcl/common/projection_matrix.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>

#include <boost/format.hpp>

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

using namespace pcl::tracking;

typedef pcl::PointXYZ RefPointType;
typedef ParticleXYZRPY ParticleT;
typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;

CloudPtr cloud_pass_;
CloudPtr cloud_pass_downsampled_;
CloudPtr target_cloud;

boost::mutex mtx_;
boost::shared_ptr<ParticleFilter> tracker_;
bool new_cloud_;
double downsampling_grid_size_;
int counter;
ros::Publisher pub;


//Filter along a specified dimension
void filterPassThrough (const CloudConstPtr &cloud, Cloud &result)
{
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 10.0);
  pass.setKeepOrganized (false);
  pass.setInputCloud (cloud);
  pass.filter (result);
}


void gridSampleApprox (const CloudConstPtr &cloud, Cloud &result, double leaf_size)
{
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> grid;
  grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
  grid.setInputCloud (cloud);
  grid.filter (result);
}


//Draw the current particles
bool drawParticles (pcl::visualization::PCLVisualizer& viz)
{
  ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles ();
  if (particles && new_cloud_)
    {
      //Set pointCloud with particle's points
      pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
      for (size_t i = 0; i < particles->points.size (); i++)
    {
      pcl::PointXYZ point;
          
      point.x = particles->points[i].x;
      point.y = particles->points[i].y;
      point.z = particles->points[i].z;
      particle_cloud->points.push_back (point);
    }

      //Draw red particles 
      {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color (particle_cloud, 250, 99, 71);

    if (!viz.updatePointCloud (particle_cloud, red_color, "particle cloud"))
      viz.addPointCloud (particle_cloud, red_color, "particle cloud");
      }
      return true;
    }
  else
    {
      return false;
    }
}

//Draw model reference point cloud
void drawResult (pcl::visualization::PCLVisualizer& viz)
{
  ParticleXYZRPY result = tracker_->getResult ();
  Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);

  //move close to camera a little for better visualization
  transformation.translation () += Eigen::Vector3f (0.0f, 0.0f, -0.005f);
  CloudPtr result_cloud (new Cloud ());
  pcl::transformPointCloud<RefPointType> (*(tracker_->getReferenceCloud ()), *result_cloud, transformation);

  //Draw blue model reference point cloud
  {
    pcl::visualization::PointCloudColorHandlerCustom<RefPointType> blue_color (result_cloud, 0, 0, 255);

    if (!viz.updatePointCloud (result_cloud, blue_color, "resultcloud"))
      viz.addPointCloud (result_cloud, blue_color, "resultcloud");
  }
}

//visualization's callback function
void viz_cb (pcl::visualization::PCLVisualizer& viz)
{
  // boost::mutex::scoped_lock lock (mtx_);
    
  // if (!cloud_pass_)
  //   {
  //     boost::this_thread::sleep (boost::posix_time::seconds (1));
  //     return;
  //  }

  //Draw downsampled point cloud from sensor    
  if (cloud_pass_downsampled_)
    {
      CloudPtr cloud_pass;
      cloud_pass = cloud_pass_downsampled_;
    
      if (!viz.updatePointCloud (cloud_pass, "cloudpass"))
    {
      viz.addPointCloud (cloud_pass, "cloudpass");
      viz.resetCameraViewpoint ("cloudpass");
    }
      bool ret = drawParticles (viz);
      if (ret)
        drawResult (viz);
    }
  new_cloud_ = false;
}


void cloudcallback (const my_new_msgs::clustering& msg ){

    sensor_msgs::PointCloud2 accumulator;

    sensor_msgs::PointCloud2 cluster_msgs;

    for (size_t i=0; i< msg.clusters.size(); i++){

        pcl::PCLPointCloud2 cloud2;
        pcl_conversions::toPCL( msg.clusters[i] , cloud2);

        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ> ());
        pcl::fromPCLPointCloud2(cloud2, *target_cloud);

        // target_cloud.reset(new Cloud());
        pcl::PCLPointCloud2 newcloud2;
        pcl::toPCLPointCloud2(*target_cloud, newcloud2);
        pcl_conversions::fromPCL(newcloud2, cluster_msgs);
        sensor_msgs::PointCloud2 tmp = sensor_msgs::PointCloud2(accumulator);
        pcl::concatenatePointCloud( cluster_msgs, tmp, accumulator);

        pcl::PCLPointCloud2 pcl2;
        pcl_conversions::toPCL( accumulator , pcl2);

        //target_cloud.reset(new Cloud());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass_(new pcl::PointCloud<pcl::PointXYZ> ());
        pcl::fromPCLPointCloud2(pcl2, *cloud_pass_);
    }

    //prepare the model of tracker's target
    for (size_t j=0; msg.clusters.size(); j=0){

        pcl::PCLPointCloud2 cloud2;
        pcl_conversions::toPCL( msg.clusters[j] , cloud2);


        // pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ> ());
        target_cloud.reset(new Cloud());
        pcl::fromPCLPointCloud2(cloud2, *target_cloud);

        Eigen::Vector4f c;
        Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
        CloudPtr transed_ref (new Cloud);
        CloudPtr transed_ref_downsampled (new Cloud);

        pcl::compute3DCentroid<RefPointType> (*target_cloud, c);

        trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);

        pcl::transformPointCloud<RefPointType> (*target_cloud, *transed_ref, trans.inverse());

        gridSampleApprox (transed_ref, *transed_ref_downsampled, downsampling_grid_size_);


        //set reference model and trans
        tracker_->setReferenceCloud (transed_ref_downsampled);

        tracker_->setTrans (trans);
    }

    CloudConstPtr cloud;
    // cloud_pass_.reset (new Cloud);
    cloud_pass_downsampled_.reset (new Cloud);
;
    filterPassThrough (cloud, *cloud_pass_);
    gridSampleApprox (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);
;

         //Track the object
    tracker_->setInputCloud (cloud_pass_downsampled_);
;

    //tracker_->compute ();
;
    new_cloud_ = true;

}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "tracking");
    ros::NodeHandle n_;

    ros::Subscriber sub = n_.subscribe ("/new_pcl", 1, cloudcallback);

    //Setup OpenNIGrabber and viewer
    pcl::visualization::CloudViewer* viewer_ = new pcl::visualization::CloudViewer("PCL OpenNI Tracking Viewer");
    // pcl::Grabber* interface = new pcl::OpenNIGrabber (device_id);
    // boost::function<void (const CloudConstPtr&)> f =
    //   boost::bind (&cloud_cb, _1);
    // interface->registerCallback (f);
      
    viewer_->runOnVisualizationThread (boost::bind(&viz_cb, _1), "viz_cb");

    //Start viewer and object tracking
    // interface->start();

    //interface->stop();
 //Set parameters
    new_cloud_ = false;
    downsampling_grid_size_ = 0.002;

    std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
    default_step_covariance[3] *= 40.0;
    default_step_covariance[4] *= 40.0;
    default_step_covariance[5] *= 40.0;

    std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
    std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

    boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> > tracker
      (new KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> (8));

    ParticleT bin_size;
    bin_size.x = 0.1f;
    bin_size.y = 0.1f;
    bin_size.z = 0.1f;
    bin_size.roll = 0.1f;
    bin_size.pitch = 0.1f;
    bin_size.yaw = 0.1f;


    //Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
    tracker->setMaximumParticleNum (1000);
    tracker->setDelta (0.99);
    tracker->setEpsilon (0.2);
    tracker->setBinSize (bin_size);

    //Set all parameters for  ParticleFilter
    tracker_ = tracker;
    tracker_->setTrans (Eigen::Affine3f::Identity ());
    tracker_->setStepNoiseCovariance (default_step_covariance);
    tracker_->setInitialNoiseCovariance (initial_noise_covariance);
    tracker_->setInitialNoiseMean (default_initial_mean);
    tracker_->setIterationNum (1);
    tracker_->setParticleNum (600);
    tracker_->setResampleLikelihoodThr(0.00);
    tracker_->setUseNormal (false);


    //Setup coherence object for tracking
    ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence = ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr
      (new ApproxNearestPairPointCloudCoherence<RefPointType> ());
      
    boost::shared_ptr<DistanceCoherence<RefPointType> > distance_coherence
      = boost::shared_ptr<DistanceCoherence<RefPointType> > (new DistanceCoherence<RefPointType> ());
    coherence->addPointCoherence (distance_coherence);

    boost::shared_ptr<pcl::search::Octree<RefPointType> > search (new pcl::search::Octree<RefPointType> (0.01));
    coherence->setSearchMethod (search);
    coherence->setMaximumDistance (0.01);

    tracker_->setCloudCoherence (coherence);


    while (ros::ok() /*&& !viewer_->wasStopped ()*/){
      ros::spinOnce();
      //boost::this_thread::sleep(boost::posix_time::seconds(1));
      //ros::sleep(ros::Duration(1));
    }
}