#!/usr/bin/env python
# coding: utf-8
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

import pcl
import pcl_helper

def do_statistical_outlier_filtering(pcl_data,mean_k,tresh):
    '''
    :param pcl_data: point cloud data subsrcriber
    :param mean_k: number of neighboring points to analysis for any given point
    :param tresh: Any point with a mean distance larger than global will be considered outlier
    :return : Statistical outlier filtered point cloud data
    eg) cloud = do_statistical_outlier_filtering(cloud,10,0.001)
    : https://github.com/fouliex/RoboticsPerception
    '''
    outlier_filter = pcl_data.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(mean_k)
    outlier_filter.set_std_dev_mul_thresh(tresh)
    return outlier_filter.filter()
    
def callback(input_ros_msg):
    cloud = pcl_helper.ros_to_pcl(input_ros_msg)
    print("Input :",cloud,type(cloud))
    
    #executing part
    cloud = pcl_helper.XYZRGB_to_XYZ(cloud)
    
    mean_k = 10
    tresh = 0.001
    cloud = do_statistical_outlier_filtering(cloud,mean_k,tresh)
    
    color = pcl_helper.random_color_gen()
    cloud = pcl_helper.XYZ_to_XYZRGB(cloud,color)
    print("Output :", cloud, type(cloud))
    print("")
    
    cloud_new = pcl_helper.pcl_to_ros(cloud)      #convert PCL to ROS msg
    pub.publish(cloud_new)
    
if __name__ == "__main__":
    rospy.init_node('tutorial',anonymous=True)
    rospy.Subscriber('/laserPointCloud',PointCloud2,callback)
    
    pub = rospy.Publisher("/laserPointCloud2",PointCloud2,queue_size=1)
    rospy.spin()
