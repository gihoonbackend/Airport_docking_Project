#!/usr/bin/env python3
# coding: utf-8

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

import pcl
import pcl_helper

def do_passthrough(pcl_data,filter_axis,axis_min,axis_max):
    '''
    Create a PassThrough object and assigns a filter axis and range.
    :param pcl_data: point could data subscriber
    :param filter_axis: filter axis
    :param axis_min: Minimum axis to the passthrough filter object
    :param axis_max: Maximum axis to the passthrough filter object
    :return: passthrough on point cloud
    '''
    passthrough = pcl_data.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    return passthrough.filter()

# Use RANSAC planse segmentation to separate plane and not plane points
# Returns inliers (plane) and outliers (not plane)
def do_ransac_line_segmentation(point_cloud, input_max_distance):
    #segmenter = point_cloud.make_segmenter_normals(ksearch=50)
    segmenter = point_cloud.make_segmenter()
    #segmenter.set_optimize_coefficients(True)
    segmenter.set_model_type(pcl.SACMODEL_LINE)  #pcl_sac_model_line
    #segmenter.set_normal_distance_weight(0.1)
    segmenter.set_method_type(pcl.SAC_RANSAC) #pcl_sac_ransac
    #segmenter.set_max_iterations(1000)
    segmenter.set_distance_threshold(input_max_distance) #0.03) #max_distance #RANSAC distance threshold
    indices, coefficients = segmenter.segment()

    inliers = point_cloud.extract(indices, negative=False)
    outliers = point_cloud.extract(indices, negative=True)
    print("indices: ",indices)
    print("coefficients: ",coefficients)

    return indices, inliers, outliers

def callback(input_ros_msg):

    cloud = pcl_helper.ros_to_pcl(input_ros_msg)
    print("Input :", cloud, type(cloud))

    # executing part
    cloud = do_passthrough(cloud, 'x', 1.0, 20.0)
    cloud = do_passthrough(cloud, 'y', -7.0, 5.5)
    _, _, cloud = do_ransac_line_segmentation(cloud, 0.05)

    cloud_new = pcl_helper.pcl_to_ros(cloud) #PCL을 ROS 메시지로 변경     
    pub.publish(cloud_new)

if __name__ == "__main__":
    rospy.init_node('RANSAC', anonymous=True)
    #rospy.Subscriber("/pcl_cluster", PointCloud2, queue_size = 1)
    rospy.Subscriber("/laserPointCloud", PointCloud2, queue_size = 1)

    pub = rospy.Publisher("/Cluster_RANSAC", PointCloud2, queue_size=1)
    
    print("success")

    rospy.spin()
