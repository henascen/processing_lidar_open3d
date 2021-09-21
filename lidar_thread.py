import time
import threading
import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import glob
from pathlib import Path
import matplotlib.pyplot as plt
import random

sensor_readings_path = Path.cwd() / 'sensors' / 'data_1'
sensor_readings_files = glob.glob(f'{str(sensor_readings_path)}/*.pcd')
sensor_readings_files = sorted(sensor_readings_files)

def main():
    gui.Application.instance.initialize()

    window = gui.Application.instance.create_window('img', width=640, height=480)
    widget = gui.SceneWidget()
    widget.scene = rendering.Open3DScene(window.renderer)
    widget.scene.set_background([1, 1, 1, 1])
    bbox = o3d.geometry.AxisAlignedBoundingBox([-10, -10, -10], [10, 10, 10])
    widget.setup_camera(60.0, bbox, [0, 0, -100])
    widget.scene.show_axes(True)
    window.add_child(widget)

    mat = rendering.Material()
    mat.shader = 'defaultLit'

    box_mat = rendering.Material()
    box_mat.shader = 'defaultLit'
    box_mat.line_width = 500

    #widget.look_at([0, 0, 0], [1, 1, 1], [0, 0, 1])

    lidar_pcd = o3d.io.read_point_cloud(sensor_readings_files[0])
    widget.scene.add_geometry('pcd', lidar_pcd, mat)

    def update_geometry(lidar_pcd_road, lidar_pcd_obstacle, bounding_boxes, image):
        if widget.scene.has_geometry('pcd'):
            widget.scene.remove_geometry('pcd')

        #if widget.scene.has_geometry('road'):
        #    widget.scene.remove_geometry('road')
        
        if widget.scene.has_geometry('obstacle'):
            widget.scene.remove_geometry('obstacle')

        #widget.scene.add_geometry('road', lidar_pcd_road, mat)
        widget.scene.add_geometry('obstacle', lidar_pcd_obstacle, mat)
        for i, bbox in enumerate(bounding_boxes):
            if widget.scene.has_geometry(f'bbox_{i}'):
                widget.scene.remove_geometry(f'bbox_{i}')
            bbox.color = np.array([0.8, 0.1, 0.1])
            widget.scene.add_geometry(f'bbox_{i}', bbox, box_mat)
        
        widget.force_redraw()

        def _on_image(image):
            name = time.time()
            o3d.io.write_image(f"./render/{name}_rendered.jpg", image)
            
        widget.scene.scene.render_to_image(_on_image)
        #print("Exporting image")
    
    def thread_main():
        image=0

        for sensor_reading in sensor_readings_files:
            lidar_pcd = o3d.io.read_point_cloud(sensor_reading)
            lidar_pcd = lidar_pcd.voxel_down_sample(voxel_size=0.2)
            bbox = o3d.geometry.AxisAlignedBoundingBox(
                min_bound=(-15, -5, -2), 
                max_bound=(30, 7, 1)
            )
            lidar_pcd = lidar_pcd.crop(bbox)
            roof_bbox = o3d.geometry.AxisAlignedBoundingBox(
                min_bound=(-1.5, -1.7, -1.0), 
                max_bound=(2.6, 1.7, -0.4)
            )
            roof_pcd = lidar_pcd.crop(roof_bbox)
            croppcd_points = np.asarray(lidar_pcd.points)
            roofpcd_points = np.asarray(roof_pcd.points)

            indices = []
            for roof_element in roofpcd_points:
                array_comparison = np.equal(croppcd_points, roof_element)
                for index, array in enumerate(array_comparison):
                    if np.sum(np.logical_and(array, [True, True, True])) == 3:
                        indices.append(index)

            lidar_pcd = lidar_pcd.select_by_index(indices, invert=True)
            
            plane_model, inliers = lidar_pcd.segment_plane(
                distance_threshold=0.2,
                ransac_n=3,
                num_iterations=100
            )
            inlier_cloud = lidar_pcd.select_by_index(inliers)
            inlier_cloud.paint_uniform_color([0.3, 0.2, 0.1])
            outlier_cloud = lidar_pcd.select_by_index(inliers, invert=True)

            with o3d.utility.VerbosityContextManager(
                    o3d.utility.VerbosityLevel.Debug) as cm:
                labels = np.array(
                    outlier_cloud.cluster_dbscan(
                        eps=0.46, 
                        min_points=10, 
                        print_progress=True)
                )

            max_label = labels.max()
            
            colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
            colors[labels < 0] = 0
            outlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
            bounding_boxes = []
            for cluster_number in list(np.unique(labels))[1:]:
                cluster_indices = np.where(labels == cluster_number)
                cluster_cloud_points = outlier_cloud.select_by_index(
                    cluster_indices[0].tolist()
                )
                object_bbox = cluster_cloud_points.get_axis_aligned_bounding_box()
                bounding_boxes.append(object_bbox)

            image += 1            
            # Update geometry
            gui.Application.instance.post_to_main_thread(
                window, lambda: update_geometry(
                    inlier_cloud, 
                    outlier_cloud,
                    bounding_boxes,
                    image))
            print(f"Displaying: {sensor_reading}")
            time.sleep(0.01)

        
    threading.Thread(target=thread_main).start()

    gui.Application.instance.run()

if __name__ == "__main__":
    main()