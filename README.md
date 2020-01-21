# Point Cloud Object Detection
See object_detection_report.pdf for an in depth explanation.

This package is split into 4 nodes: 

distance_crop.cpp
Passthrough filter. Removes points outside of bounds in Y and Z camera axes. Subscribes to /camera/depth/color/points. Publishes cropped cloud to /cropped_points.

segment_cloud.cpp
Plane segmentation and euclidean clustering. Finds all planes that satisfy the equation ax + by + cz = d (where a, b, c, and d are set to be equivalent to a plane parallel with the ground) and removes them from the point cloud so they are not detected as objstacles. Then groups clusters of points that are within some distance (set in code) of eachother as one obstacle. Subscribes to /cropped_points. Publishes segmented points to /seg_points, obstacle centroids and min/max points to /obj_info_raw, and obstacle rViz markers to /seg_markers. Also has the beginnings of code for Kalman filtering of the obstacles.

tf_seg_info.py
Transforms the segmented obstacles from the camera frame (attached to the TurtleBot) to the Earth fixed frame. Subscribes to /obj_info_raw, and /om_with_tb3/odom. Publishes to /obj_info_tf.

obj_plotter.py
Not part of the detection algorithm but plots the detected obstacles centroids and bounding boxes over time. This is not a very good interface to use for debugging/working with the algorithm and was primarily for demo purposes. rViz should be used for most cases. Subscribes to /obj_info_tf, and \om_with_tb3\odom.

