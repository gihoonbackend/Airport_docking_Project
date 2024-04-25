#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
import pcl_helper
import filtering_helper

class pattern_matcher():
    def __init__(self):
        self.laserProj=LaserProjection()
        self.laserSub=rospy.Subscriber("/scan",LaserScan,self.laserCallback)
        #self.pattern_pub=rospy.Publisher("/pattern",pc2,queue_size=1)
        self.clustered_pub=rospy.Publisher("/clustered_points",pc2,queue_size=1)
        
    def laserCallback(self,data):
    
        ##convert sensor_msgs/LaserScan to PCL_Cloud
        pointcloud=self.laserProj.projectLaser(data)
        
        ##Apply voxel filter
        #voxel = do_voxel_grid_filter(point_cloud, LEAF_SIZE = 0.01)
        
        
        ##set filtered cloud as input for KD Tree
        
        
        ## Extract clusters
        
        """
        For Every Cluster
        1. Give different color
        2. Run ICP on it
        3. Check if ICP result is the best
        """
        ## Convert ROS msg to PCL data
        #cloud = pcl_helper.ros_to_pcl(pcl_msg) 

        # Extract objects and table from the scene
        vg = filtering_helper.do_voxel_grid_filter(pointcloud,0.01) 

        # Get a point cloud of only the position information without color information
        colorless_cloud = pcl_helper.XYZRGB_to_XYZ(pointcloud)
  
        # Get groups of indices for each cluster of points
        # Each group of points belongs to the same object
        # This is effectively a list of lists, with each list containing indices of the cloud
        clusters = filtering_helper.get_clusters(colorless_cloud, tolerance = 0.05, min_size = 100, max_size = 1500)

        # Assign a unique color float for each point (x, y, z)
        # Points with the same color belong to the same cluster
        colored_points = filtering_helper.get_colored_clusters(clusters, colorless_cloud)

        # Create a cloud with each cluster of points having the same color
        clusters_cloud = pcl.PointCloud_PointXYZRGB()
        clusters_cloud.from_list(colored_points)

        # Convert pcl data to ros messages
        clusters_msg = pcl_helper.pcl_to_ros(clusters_cloud)

        # Publish ROS messages
        #laserProj=LaserProjection()
    
        #clusters_out=laserProj.projectLaser(clusters_msg)
        
        #clusters_publisher.publish(clusters_out)
        self.clustered_pub.publish(clusters_msg)
        

if __name__ == '__main__':
  #open the pcd file
    rospy.init_node('autodocking', anonymous=True)
    pm = pattern_matcher()
  
    while not rospy.is_shutdown():
        rospy.spin()
