import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


print("Load a pcd point cloud, print it, and render it")
pcd = o3d.io.read_point_cloud("/home/henry/2021/lidar-detection/open3d_test_lidar/sensors/data_1/0000000000.pcd")
print(pcd)
print(np.asarray(pcd.points))
o3d.visualization.draw_geometries(
    [pcd],
)

"""
FIRST STEP
* Filter the initial point cloud, so that the resulting point cloud 
* is ready for further processing.
* It reduces number of points in the point cloud using voxel grid filtering, 
* crops the points that are outside
* of the region of interest, and removes points that belongs 
* to the ego car's roof.
"""

# Voxel filtering

print("Downsample the point cloud with a voxel of 0.2")
downpcd = pcd.voxel_down_sample(voxel_size=0.2)
o3d.visualization.draw_geometries(
    [downpcd]
)

# Crop the points outside ROI

print("Cropping the image to only left ROI")
bbox = o3d.geometry.AxisAlignedBoundingBox(
    min_bound=(-15, -5, -2), 
    max_bound=(30, 7, 1)
    )
croppcd = downpcd.crop(bbox)
o3d.visualization.draw_geometries(
    [croppcd]
)

print("Getting the roff points")
roof_bbox = o3d.geometry.AxisAlignedBoundingBox(
    min_bound=(-1.5, -1.7, -1.0), 
    max_bound=(2.6, 1.7, -0.4)
    )
roofpcd = croppcd.crop(roof_bbox)
o3d.visualization.draw_geometries(
    [roofpcd]
)

croppcd_points = np.asarray(croppcd.points)
roofpcd_points = np.asarray(roofpcd.points)

indices = []
for roof_element in roofpcd_points:
    array_comparison = np.equal(croppcd_points, roof_element)
    for index, array in enumerate(array_comparison):
        if np.sum(np.logical_and(array, [True, True, True])) == 3:
            indices.append(index)

print("Extracting the points that doesn't belong to the roof")
regionpcd = croppcd.select_by_index(indices, invert=True)
o3d.visualization.draw_geometries(
    [regionpcd]
)

print(regionpcd)
"""
SECOND STEP

* Separates the road from obstacles.
USING THE RANSAC Algorithm
"""

print("Segmententation of geometric primitives from point clouds using RANSAC.")
plane_model, inliers = regionpcd.segment_plane(
    distance_threshold=0.2,
    ransac_n=3,
    num_iterations=100
)

inlier_cloud = regionpcd.select_by_index(inliers)
inlier_cloud.paint_uniform_color([1.0, 0, 0])
outlier_cloud = regionpcd.select_by_index(inliers, invert=True)
o3d.visualization.draw_geometries(
    [inlier_cloud, outlier_cloud],
)

print(outlier_cloud)
"""
THIRD STEP

* Split the input cloud to several clusters based on the DBScan clustering algorithm.
"""

print("Clustering using DBSCAN from a point cloud")
with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
    labels = np.array(
        outlier_cloud.cluster_dbscan(eps=0.7, min_points=10, print_progress=True)
    )

print(labels.shape)
max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")

colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
outlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
o3d.visualization.draw_geometries(
    [inlier_cloud, outlier_cloud],
)

"""
FOURTH STEP

* Determine coordinates of a bounding box around the point cloud.
"""
print(labels.min())
bounding_boxes = []
for cluster_number in list(np.unique(labels))[1:]:
    cluster_indices = np.where(labels == cluster_number)
    cluster_cloud_points = outlier_cloud.select_by_index(
        cluster_indices[0].tolist()
        )
    object_bbox = cluster_cloud_points.get_axis_aligned_bounding_box()
    bounding_boxes.append(object_bbox)

print(len(bounding_boxes))
o3d.visualization.draw_geometries(
    [inlier_cloud, outlier_cloud, *bounding_boxes]
)
