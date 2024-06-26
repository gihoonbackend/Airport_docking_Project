#!/usr/bin/env python3
# coding: utf-8
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from laser_geometry import LaserProjection
import pcl_helper
import pcl
import filtering_helper
import rviz_visualizer as visual
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
import tf2_ros
import tf2_geometry_msgs

laserProj = LaserProjection()

def do_voxel_grid_downsampling(pcl_data, leaf_size):
    vox = pcl_data.make_voxel_grid_filter()
    vox.set_leaf_size(leaf_size, leaf_size, leaf_size)  # The bigger the leaf size the less information retained
    return vox.filter()

def get_clusters(cloud, tolerance, min_size, max_size):
    tree = cloud.make_kdtree()
    extraction_object = cloud.make_EuclideanClusterExtraction()
    extraction_object.set_ClusterTolerance(tolerance)  # Set distance information considered as one cluster
    extraction_object.set_MinClusterSize(min_size)  # Minimum point number
    extraction_object.set_MaxClusterSize(max_size)  # Maximum point number
    extraction_object.set_SearchMethod(tree)  # Specify the searching method defined above
    clusters = extraction_object.Extract()  # Output: cluster_indices
    return clusters

def get_colored_clusters(clusters, cloud):
    number_of_clusters = len(clusters)
    colors = pcl_helper.get_color_list(number_of_clusters)
    colored_points = []
    for cluster_id, cluster in enumerate(clusters):
        for c, i in enumerate(cluster):
            x, y, z = cloud[i][0], cloud[i][1], cloud[i][2]
            color = pcl_helper.rgb_to_float(colors[cluster_id])
            colored_points.append([x, y, z, color])
    return colored_points

def do_euclidean_clustering(white_cloud, tolerance, min_size, max_size):
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(tolerance)
    ec.set_MinClusterSize(min_size)
    ec.set_MaxClusterSize(max_size)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    cluster_color = pcl_helper.get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             pcl_helper.rgb_to_float(cluster_color[j])])
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    return cluster_cloud, cluster_indices

def do_passthrough(pcl_data, filter_axis, axis_min, axis_max):
    passthrough = pcl_data.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    return passthrough.filter()

def do_ransac_line_segmentation(point_cloud, input_max_distance):
    segmenter = point_cloud.make_segmenter()
    segmenter.set_model_type(pcl.SACMODEL_LINE)
    segmenter.set_method_type(pcl.SAC_RANSAC)
    segmenter.set_distance_threshold(input_max_distance)
    inlires, coefficients = segmenter.segment()
    inlier_object = point_cloud.extract(inlires, negative=False)
    outlier_object = point_cloud.extract(inlires, negative=True)
    return inlier_object, outlier_object, coefficients

def find_max_x_point(cloud):
    max_x = -np.inf
    max_x_point = None
    for point in cloud:
        x = point[0]
        if x > max_x:
            max_x = x
            max_x_point = point
    return max_x_point

def publish_point_marker(point):
    marker = Marker()
    marker.header.frame_id = "map"  # Replace with your point cloud frame ID
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.scale.x = 0.05  # Marker width
    marker.scale.y = 0.05  # Marker height
    marker.scale.z = 0.05  # Marker depth

    # Set the position of the marker
    marker.pose.position.x = point[0]
    marker.pose.position.y = point[1]
    marker.pose.position.z = point[2]

    # Set the color of the marker
    marker.color.a = 1.0  # Alpha (transparency)
    marker.color.r = 1.0  # Red
    marker.color.g = 0.0  # Green
    marker.color.b = 0.0  # Blue

    return marker

def pcl_callback(pcl_msg):
    global tf_buffer

    cloud = pcl_helper.ros_to_pcl(pcl_msg)
    LEAF_SIZE = 0.05
    cloud_cut1 = do_passthrough(cloud, 'x', 0.0, 0.5)
    cloud_cut2 = do_passthrough(cloud_cut1, 'y', -1.0, 1.0)
    cloud_cut3 = pcl_helper.XYZRGB_to_XYZ(cloud_cut2)
    cluster_cloud, cluster_indices = do_euclidean_clustering(cloud_cut3, tolerance=0.1, min_size=10, max_size=100)
    max_distance = 0.018
    cloud_station, cloud_column, coefficients = do_ransac_line_segmentation(cluster_cloud, max_distance)
    max_x_point = find_max_x_point(cloud_column.to_array())

    # Transform the point to the map frame
    point_stamped = tf2_geometry_msgs.PointStamped()
    point_stamped.header.frame_id = "base_scan"  # Replace with the frame ID of your input cloud
    point_stamped.point.x = max_x_point[0]
    point_stamped.point.y = max_x_point[1]
    point_stamped.point.z = max_x_point[2]

    try:
        transform = tf_buffer.lookup_transform("map", "base_scan", rospy.Time(0), rospy.Duration(1.0))  # Replace "map" with your target frame
        transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
        marker = publish_point_marker([transformed_point.point.x, transformed_point.point.y, transformed_point.point.z])
        # Print the transformed point coordinates
        rospy.loginfo(f"Transformed top point: x={transformed_point.point.x}, y={transformed_point.point.y}, z={transformed_point.point.z}")
    except tf2_ros.LookupException as e:
        rospy.logerr(f"Transform lookup failed: {e}")
        return
    except tf2_ros.ExtrapolationException as e:
        rospy.logerr(f"Transform extrapolation failed: {e}")
        return

    ros_cloud_cut2 = pcl_helper.pcl_to_ros(cloud_cut2)
    ros_cloud_cut3 = pcl_helper.pcl_to_ros(cluster_cloud)
    ros_cloud_station = pcl_helper.pcl_to_ros(cloud_station)
    ros_cloud_column = pcl_helper.pcl_to_ros(cloud_column)

    pcl_station_pub.publish(ros_cloud_station)
    pcl_column_pub.publish(ros_cloud_column)
    pcl_cloud_cut_pub1.publish(ros_cloud_cut2)
    pcl_cloud_cut_pub2.publish(ros_cloud_cut3)
    marker_pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node('clustering', anonymous=True)

    global tf_buffer
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    subscriber = rospy.Subscriber('/laserPointCloud', PointCloud2, pcl_callback, queue_size=1)

    pcl_cloud_cut_pub1 = rospy.Publisher("/pcl_cloud_cut2", PointCloud2, queue_size=1)
    pcl_cloud_cut_pub2 = rospy.Publisher("/cluster_cloud", PointCloud2, queue_size=1)

    pcl_station_pub = rospy.Publisher("/pcl_station", PointCloud2, queue_size=1)
    pcl_column_pub = rospy.Publisher("/pcl_column", PointCloud2, queue_size=1)

    marker_pub = rospy.Publisher('Top_point', Marker, queue_size=10)

    pcl_helper.get_color_list.color_list = []

    while not rospy.is_shutdown():
        rospy.spin()
