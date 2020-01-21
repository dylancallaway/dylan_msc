# Point Cloud Object Detection
This package takes in point cloud data from an Intel Realsense camera and outputs segmented clusters of points that are deemed objects. See object_detection_report.pdf for a more in depth explanation of how this package works.

This package is split into 4 nodes: 

distance_crop.cpp
Passthrough filter. Removes points outside of bounds (set in code) in Y and Z camera axes. Subscribes to /camera/depth/color/points. Publishes cropped cloud to /cropped_points.

segment_cloud.cpp:
Plane segmentation and euclidean clustering. Finds all planes that satisfy the equation ax + by + cz = d (where a, b, c, and d are set to be equivalent to a plane parallel with the ground) (set in code) and removes them from the point cloud so they are not detected as objstacles. Then groups clusters of points that are within some distance (set in code) of eachother as one obstacle. Subscribes to /cropped_points. Publishes segmented points to /seg_points, obstacle centroids and min/max points to /obj_info_raw, and obstacle rViz markers to /seg_markers. Also has the beginnings of code for Kalman filtering of the obstacles. The custom message obj.msg is used to store the centroid and min/max points of each cluster.

tf_seg_info.py:
Transforms the segmented obstacles from the camera frame (attached to the TurtleBot) to the Earth fixed frame. Subscribes to /obj_info_raw, and /om_with_tb3/odom. Publishes to /obj_info_tf. The custom message obj.msg is used to store the centroid and min/max points of each cluster.

obj_plotter.py:
Not part of the detection algorithm but plots the detected obstacles centroids and bounding boxes over time. This is not a very good interface to use for debugging/working with the algorithm and was primarily for demo purposes. rViz should be used for most cases. Subscribes to /obj_info_tf, and /om_with_tb3/odom.

msi.launch starts the segment_cloud, tf_seg_info, and obj_plotter nodes and should be run on the computer that acts as the Master Node (if the TurtleBot is not the Master Node).

turtlebot.launch starts the realsense2_camera, and distance_crop nodes and should be run on the TurtleBot. The camera settings can be edited in this launch file.

This package relies on the PCL package (http://www.pointclouds.org/) (which is included in the full install of ROS Kinetic) and the Intel Realsense ROS package and drivers (https://github.com/IntelRealSense/realsense-ros).

The CMakeLists.txt and package.xml are set up to include the required packages and can be inspected for how to set up other similar packages or packages that use this package.
