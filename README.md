# Obstacle detection using Open3D.

![detection_gif](./results/data_1.gif)

This repository aims to be useful guide on how you can perform point cloud processing using the Open3D library in Python, with scripts and with notebooks. The processing here used is a simple but useful analysis of pcd files to extract information.

Files:
- render: final state of processing for each lidar reading.
- sensors/data_1: pcd files (lidar readings)
- poetry.lock: necessary packages to run the example
- processing.py: step-by-step processing script
- lidar_thread.py: using the Open3D's Application GUI we create a visualization window to see the whole sensor reading and processing sequence
- lidar_processing.ipynb: step-by-step processing python notebook


