#!/usr/bin/env python3
# coding: utf-8
import rospy
import tf
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import ColorRGBA
import sensor_msgs.point_cloud2 as pc2
from laser_geometry import LaserProjection
import pcl_helper
import pcl

laserProj = LaserProjection()

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
    inliers, coefficients = segmenter.segment()

    if not inliers:
        raise Exception("no.")

    inlier_object = point_cloud.extract(inliers, negative=False)
    outlier_object = point_cloud.extract(inliers, negative=True)

    return inlier_object, outlier_object, coefficients, inliers

def find_line_end_points(cloud):
    min_y = float('inf')
    min_point = [float('inf'), float('inf'), float('inf')]
    max_y = -float('inf')
    max_point = [-float('inf'), -float('inf'), -float('inf')]
    
    total_x = 0
    point_count = 0
    
    for point in pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
        x, y, z = point
        min_y = min(min_y, y)
        max_y = max(max_y, y)
        
        total_x += x
        point_count += 1
    
    avg_x = total_x / point_count
    
    min_point[0] = avg_x
    min_point[1] = min_y
    min_point[2] = 0.0
    
    max_point[0] = avg_x
    max_point[1] = max_y
    max_point[2] = 0.0
    
    return min_point, max_point

def publish_line_marker(coefficients, min_point, max_point, listener):
    mid_point = [(min_point[i] + max_point[i]) / 2 for i in range(3)]
    
    marker = Marker()
    marker.header.frame_id = "base_scan"
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.02
    marker.pose.orientation.w = 1.0
    
    point1 = Point(min_point[0], min_point[1], min_point[2])
    point2 = Point(mid_point[0], mid_point[1], mid_point[2])
    marker.points.append(point1)
    marker.points.append(point2)
    
    marker.colors.append(ColorRGBA(1.0, 0.0, 0.0, 1.0))
    marker.colors.append(ColorRGBA(0.0, 1.0, 0.0, 1.0))
    
    point3 = Point(mid_point[0], mid_point[1], mid_point[2])
    point4 = Point(max_point[0], max_point[1], max_point[2])
    marker.points.append(point3)
    marker.points.append(point4)
    
    marker.colors.append(ColorRGBA(0.0, 1.0, 0.0, 1.0))
    marker.colors.append(ColorRGBA(1.0, 0.0, 0.0, 1.0))

    point7 = Point(mid_point[0] + 0.08, mid_point[1], mid_point[2])
    point8 = Point(mid_point[0] - 0.2, mid_point[1], mid_point[2])
    marker.points.append(point7)
    marker.points.append(point8)

    bottom_point = find_bottom_point(point7, point8)
    
    marker.colors.append(ColorRGBA(1.0, 1.0, 0.0, 1.0))
    marker.colors.append(ColorRGBA(1.0, 1.0, 0.0, 1.0))
    
    marker_pub.publish(marker)
    
    arrive_point_marker = Marker()
    arrive_point_marker.header.frame_id = "base_scan"
    arrive_point_marker.type = Marker.SPHERE
    arrive_point_marker.action = Marker.ADD
    arrive_point_marker.scale.x = 0.05
    arrive_point_marker.scale.y = 0.05
    arrive_point_marker.scale.z = 0.05
    arrive_point_marker.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)
    arrive_point_marker.pose.position.x = point7.x
    arrive_point_marker.pose.position.y = point7.y
    arrive_point_marker.pose.position.z = point7.z
    
    arrive_point_pub.publish(arrive_point_marker)
    
    arrive_point_msg = PointStamped()
    arrive_point_msg.header.frame_id = "base_scan"
    arrive_point_msg.point.x = point7.x
    arrive_point_msg.point.y = point7.y
    arrive_point_msg.point.z = point7.z
    
    try:
        listener.waitForTransform("map", "base_scan", rospy.Time(0), rospy.Duration(4.0))
        transformed_point = listener.transformPoint("map", arrive_point_msg)
        
        arrive_point_map_pub.publish(transformed_point.point)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("TF 변환 실패")
    
    return marker

def find_bottom_point(point1, point2):
    if point1.x < point2.x:
        return point1
    else:
        return point2

def pcl_callback(pcl_msg):
    max_distance = 0.018

    cloud = pcl_helper.ros_to_pcl(pcl_msg)
    cloud_cut1 = do_passthrough(cloud, 'x', 0.0, 0.5)
    cloud_cut2 = do_passthrough(cloud_cut1, 'y', -1.0, 1.0)
    cloud_cut3 = pcl_helper.XYZRGB_to_XYZ(cloud_cut2)

    cluster_cloud, cluster_indices = do_euclidean_clustering(cloud_cut3, tolerance=0.1, min_size=10, max_size=100)

    try:
        cloud_station, cloud_column, coefficients, inliers = do_ransac_line_segmentation(cluster_cloud, max_distance)
        cloud_station_msg = pcl_helper.pcl_to_ros(cloud_station)
        min_point, max_point = find_line_end_points(cloud_station_msg)
        marker = publish_line_marker(coefficients, min_point, max_point, listener)
        marker_pub.publish(marker)
    except Exception as e:
        rospy.logerr('RANSAC segmentation failed or no valid line found: {}'.format(e))
        return

if __name__ == '__main__':
    rospy.init_node('clustering', anonymous=True)
    
    listener = tf.TransformListener()
    
    subscriber = rospy.Subscriber('/laserPointCloud', PointCloud2, pcl_callback, queue_size=1)
    
    pcl_cloud_cut_pub1 = rospy.Publisher("/pcl_cloud_cut2", PointCloud2, queue_size=1)
    pcl_cloud_cut_pub2 = rospy.Publisher("/cluster_cloud", PointCloud2, queue_size=1)
    
    pcl_station_pub = rospy.Publisher("/pcl_station", PointCloud2, queue_size=1)
    pcl_column_pub = rospy.Publisher("/pcl_column", PointCloud2, queue_size=1)
    
    marker_pub = rospy.Publisher('line', Marker, queue_size=10)
    arrive_point_pub = rospy.Publisher('arrive_point', Marker, queue_size=10)
    arrive_point_map_pub = rospy.Publisher('arrive_point_map', Point, queue_size=10)
    
    pcl_helper.get_color_list.color_list = []

    while not rospy.is_shutdown():
        rospy.spin()