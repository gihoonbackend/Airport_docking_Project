import pcl

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
    
cloud=pcl.load("/home/unicon4/pcl_ws/lobby.pcd")
print(cloud)

LEAF_SIZE=0.01
cloud=do_voxel_grid_downsampling(cloud,LEAF_SIZE)
print(cloud)

LEAF_SIZE=0.1
cloud=do_voxel_grid_downsampling(cloud,LEAF_SIZE)
print(cloud)

LEAF_SIZE=1.0
cloud=do_voxel_grid_downsampling(cloud,LEAF_SIZE)
print(cloud)
