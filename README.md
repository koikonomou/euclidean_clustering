# euclidean_clustering

Clustering and Tracking package

### Prerequisites

-PCL 1.8.1 
-my_new_msgs package 
-laserscan2pcl package 

### Running the tests

**1.** Roscore & Rviz 

**2.** Rosbag : rosbag play (rosbag file) --clock -l

**3.** Launch file from *laserscan2pcl package* (convert lasercans to pointcloud) : tf_transform.launch

**4.** Launch file from *euclidean_clustering package* ( clustering ) : euclidean_clustering.launch

**5.** Launch file from *euclidean_clustering package* ( Tracking ) : Tracking.launch

**6.** Launch file from *euclidean_clustering package* ( colored clusters after tracking ) : colored_clusters.launch
