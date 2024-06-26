#!/usr/bin/env python3
# coding: utf-8
import rospy
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from laser_geometry import LaserProjection
import pcl_helper
import pcl
import filtering_helper
import rviz_visualizer as visual
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped, Twist
import tf2_ros
import tf2_geometry_msgs

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
    segmenter.set_model_type(pcl.SACMODEL_LINE)  #pcl_sac_model_line   #pcl_sac_model_plane
    segmenter.set_method_type(pcl.SAC_RANSAC) #pcl_sac_ransac
          #max_distance for a point to be considered fitting the model
    segmenter.set_distance_threshold(input_max_distance) #0.03) #RANSAC distance threshold
    inliers, coefficients = segmenter.segment()

    if not inliers:
        raise Exception("no.")

    inlier_object = point_cloud.extract(inliers, negative=False)
    outlier_object = point_cloud.extract(inliers, negative=True)

    for i in range(0, len(inliers)):      #range(0, len(inlires)):
        return inlier_object, outlier_object, coefficients, inliers

def pcl_callback(pcl_msg):
    max_distance = 0.018  # RANSAC 알고리즘에 사용할 최대 거리 임계값 설정

    cloud = pcl_helper.ros_to_pcl(pcl_msg)
    cloud_cut1 = do_passthrough(cloud, 'x', 0.0, 0.5)       
    cloud_cut2 = do_passthrough(cloud_cut1, 'y', -1.0, 1.0)
    cloud_cut3 = pcl_helper.XYZRGB_to_XYZ(cloud_cut2)

    cluster_cloud, cluster_indices = do_euclidean_clustering(cloud_cut3, tolerance=0.1, min_size=10, max_size=100)

    try:
        cloud_station, cloud_column, coefficients, inliers = do_ransac_line_segmentation(cluster_cloud, max_distance)
        cloud_station_msg = pcl_helper.pcl_to_ros(cloud_station)
        min_point, max_point = find_line_end_points(cloud_station_msg)
        marker = publish_line_marker(coefficients, min_point, max_point)
        marker_pub.publish(marker)
    except Exception as e:
        rospy.logerr('RANSAC segmentation failed or no valid line found: {}'.format(e))
        return

def find_line_end_points(cloud):
    # 최소 및 최대 y값을 찾기 위한 초기화
    min_y = float('inf')
    min_point = [float('inf'), float('inf'), float('inf')]
    max_y = -float('inf')
    max_point = [-float('inf'), -float('inf'), -float('inf')]
    
    # 클러스터 내 모든 포인트의 총 x 좌표 및 포인트 수 초기화
    total_x = 0
    point_count = 0
    
    # 모든 포인트를 반복하면서 최소 및 최대 y값을 찾고, x 좌표를 더합니다.
    for point in pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
        x, y, z = point
        min_y = min(min_y, y)
        max_y = max(max_y, y)
        
        # x 좌표 누적
        total_x += x
        point_count += 1
    
    # 선의 x 좌표를 클러스터 내 모든 포인트의 x 좌표의 평균값으로 설정
    avg_x = total_x / point_count
    
    # 최소 y값에 해당하는 포인트의 x 좌표를 클러스터의 왼쪽 끝으로 설정
    min_point[0] = avg_x
    min_point[1] = min_y
    min_point[2] = 0.0
    
    # 최대 y값에 해당하는 포인트의 x 좌표를 클러스터의 오른쪽 끝으로 설정
    max_point[0] = avg_x
    max_point[1] = max_y
    max_point[2] = 0.0
    
    return min_point, max_point

# 이 함수는 최소 및 최대 포인트를 사용하여 선분 마커를 생성합니다.
def publish_line_marker(coefficients, min_point, max_point):
    # 선의 중간점 계산
    mid_point = [(min_point[i] + max_point[i]) / 2 for i in range(3)]
    
    # 네 번째 선분: y 방향으로 중심을 통과하는 선분 (노란색)
    point7 = Point(mid_point[0], mid_point[1], mid_point[2])
    point8 = Point(mid_point[0] - 0.2, mid_point[1], mid_point[2])
    
    bottom_point = find_bottom_point(point7, point8)
    ## rospy.loginfo("Bottom point before transform: x={}, y={}, z={}".format(bottom_point.x, bottom_point.y, bottom_point.z))

    # Transform the bottom point to the map frame
    bottom_point_stamped = PointStamped()
    bottom_point_stamped.header.frame_id = "base_scan"  # Replace with the frame ID of your input cloud
    bottom_point_stamped.point = bottom_point

    try:
        transform = tf_buffer.lookup_transform("map", "base_scan", rospy.Time(0), rospy.Duration(1.0))  # Replace "map" with your target frame
        transformed_bottom_point = tf2_geometry_msgs.do_transform_point(bottom_point_stamped, transform)
        rospy.loginfo("Transformed bottom point: x={}, y={}, z={}".format(transformed_bottom_point.point.x, transformed_bottom_point.point.y, transformed_bottom_point.point.z))
    except tf2_ros.LookupException as e:
        rospy.logerr(f"Transform lookup failed: {e}")
        return
    except tf2_ros.ExtrapolationException as e:
        rospy.logerr(f"Transform extrapolation failed: {e}")
        return

    marker = Marker()
    marker.header.frame_id = "map"  # Publish in the map frame
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.pose.position = transformed_bottom_point.point
    marker.color = ColorRGBA(1.0, 1.0, 0.0, 1.0)  # Yellow

    # 마커 발행
    marker_pub.publish(marker)
    return marker

def find_bottom_point(point1, point2):
    # point1과 point2 중 x 좌표가 더 작은 점을 반환
    if point1.x < point2.x:
        return point1
    else:
        return point2

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
    
    marker_pub = rospy.Publisher('Bottom_point', Marker, queue_size=10)

    pcl_helper.get_color_list.color_list = []

    while not rospy.is_shutdown():
        rospy.spin()
