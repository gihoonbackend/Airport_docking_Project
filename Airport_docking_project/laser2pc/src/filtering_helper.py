#!/usr/bin/env python

# Import modules
import pcl
import pcl_helper

# Returns Downsampled version of a point cloud
# The bigger the leaf size the less information retained
def do_voxel_grid_filter(point_cloud, LEAF_SIZE = 0.01):
  voxel_filter = point_cloud.make_voxel_grid_filter()
  voxel_filter.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE) 
  return voxel_filter.filter()

# Returns only the point cloud information at a specific range of a specific axis
def do_passthrough_filter(point_cloud, name_axis = 'z', min_axis = 0.6, max_axis = 1.1):
  pass_filter = point_cloud.make_passthrough_filter()
  pass_filter.set_filter_field_name(name_axis);
  pass_filter.set_filter_limits(min_axis, max_axis)
  return pass_filter.filter()

# Use RANSAC planse segmentation to separate plane and not plane points
# Returns inliers (plane) and outliers (not plane)
def do_ransac_plane_segmentation(point_cloud, max_distance = 0.01):

  segmenter = point_cloud.make_segmenter()

  segmenter.set_model_type(pcl.SACMODEL_PLANE)
  segmenter.set_method_type(pcl.SAC_RANSAC)
  segmenter.set_distance_threshold(max_distance)

  #obtain inlier indices and model coefficients
  inlier_indices, coefficients = segmenter.segment()

  inliers = point_cloud.extract(inlier_indices, negative = False)
  outliers = point_cloud.extract(inlier_indices, negative = True)

  return inliers, outliers

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
