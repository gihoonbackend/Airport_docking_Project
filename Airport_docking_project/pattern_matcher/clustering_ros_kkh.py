#!/usr/bin/env python3
# coding: utf-8
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from laser_geometry import LaserProjection
import pcl_helper
import pcl
import filtering_helper

laserProj=LaserProjection()
       
def do_voxel_grid_downsampling(pcl_data,leaf_size):
    '''
    Create a VoxelGrid filter object for a input point cloud
    :param pcl_data: point cloud data subscriber
    :param leaf_size: voxel(or leaf) size
    :return : Voxel grid downsampling on point cloud
                     '''
    vox=pcl_data.make_voxel_grid_filter()
    vox.set_leaf_size(leaf_size,leaf_size,leaf_size)     #The bigger the leaf size the less information retained
    return vox.filter()
    
    # This pipeline returns groups of indices for each cluster of points
    # Each cluster of indices is grouped as belonging to the same object
    # This uses DBSCAN Algorithm Density-Based Spatial Clustering of Applications with noise
    # Aka Eucledian clustering to group points 

def get_clusters(cloud, tolerance, min_size, max_size):

    tree = cloud.make_kdtree() 
    extraction_object = cloud.make_EuclideanClusterExtraction()

    extraction_object.set_ClusterTolerance(tolerance)
    extraction_object.set_MinClusterSize(min_size)
    extraction_object.set_MaxClusterSize(max_size)
    extraction_object.set_SearchMethod(tree)

        # Get clusters of indices for each cluster of points, each clusterbelongs to the same object
        # 'clusters' is effectively a list of lists, with each list containing indices of the cloud
    clusters = extraction_object.Extract()
    return clusters
  

    # clusters is a list of lists each list containing indices of the cloud
    # cloud is an array with each cell having three numbers corresponding to x, y, z position
    # Returns list of [x, y, z, color]
def get_colored_clusters(clusters, cloud):
  
        # Get a random unique colors for each object
    number_of_clusters = len(clusters)
    colors = pcl_helper.get_color_list(number_of_clusters)

    colored_points = []

        # Assign a color for each point
        # Points with the same color belong to the same cluster
    for cluster_id, cluster in enumerate(clusters):
        for c, i in enumerate(cluster):
            x, y, z = cloud[i][0], cloud[i][1], cloud[i][2]
            color = pcl_helper.rgb_to_float(colors[cluster_id])
            colored_points.append([x, y, z, color])
  
    return colored_points

    # Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

        # Convert ROS msg to PCL data
    cloud = pcl_helper.ros_to_pcl(pcl_msg) 

        # Extract objects and table from the scene
    cloud = do_voxel_grid_downsampling(cloud,0.01) 

        # Get a point cloud of only the position information without color information
    colorless_cloud = pcl_helper.XYZRGB_to_XYZ(cloud)
  
        # Get groups of indices for each cluster of points
        # Each group of points belongs to the same object
        # This is effectively a list of lists, with each list containing indices of the cloud
    clusters = get_clusters(colorless_cloud, tolerance = 0.05, min_size = 100, max_size = 1500)

        # Assign a unique color float for each point (x, y, z)
        # Points with the same color belong to the same cluster
    colored_points = get_colored_clusters(clusters, colorless_cloud)

        # Create a cloud with each cluster of points having the same color
    clusters_cloud = pcl.PointCloud_PointXYZRGB()
    clusters_cloud.from_list(colored_points)

        # Convert pcl data to ros messages
    clusters_msg = pcl_helper.pcl_to_ros(clusters_cloud)

        # Publish ROS messages
    #laserProj=LaserProjection()
    
    #clusters_out=laserProj.projectLaser(clusters_msg)
        
    #clusters_publisher.publish(clusters_out)
    clusters_publisher.publish(clusters_msg)


if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('clustering', anonymous = True)

    # Create Subscribers
    subscriber = rospy.Subscriber('/laserPointCloud',PointCloud2,pcl_callback,queue_size=1)
    
    # Create Publishers
    clusters_publisher = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size = 1)
    #clusters_publisher = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size = 1)
  
    # Initialize color_list
    pcl_helper.get_color_list.color_list = []
    #cluster=Clustering() 
    print('success')

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
