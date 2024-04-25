#!/usr/bin/env python3
# coding: utf-8
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

import pcl_helper
import pcl
from laser_geometry import LaserProjection

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

def callback(input_ros_msg):
    cloud=pcl_helper.ros_to_pcl(input_ros_msg)
    print("Input :",cloud)
    
    LEAF_SIZE=0.1
    cloud=do_voxel_grid_downsampling(cloud,LEAF_SIZE)
    print("Output :",cloud)
    print("")
    
    cloud_new=pcl_helper.pcl_to_ros(cloud)
    
    pub.publish(cloud_new)
    
if __name__ == "__main__":
    rospy.init_node('downsampling_voxelgrid',anonymous=True)
    rospy.Subscriber('/laserPointCloud',PointCloud2,callback)
    
    pub=rospy.Publisher("/laserPointCloud_new",PointCloud2,queue_size=1)
    rospy.spin()
