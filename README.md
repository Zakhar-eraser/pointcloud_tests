# Realsense Point Cloud Tests Package
This package consists of different tests of processing PointCloud2 messages published by realsense d435 camera
## Main distances
This program finds mean values of 5 pointcloud areas: Upper, lower, center, left and right kernels.
Launch:
~~~
roslaunch pointcloud_tests mean_distances.launch
~~~
Change meaning window:
~~~
rosparam set /mean_distances/window_size 5
~~~
![Alt text](docs/pointcloud.png)
![Alt text](docs/main_points.png)
## Yaw from main distances
This program calculate yaw and pitch from point cloud
![Alt text](docs/flat_wall.png)
The picture shows how program calculates angles with flat wall
![Alt text](docs/noisy.png)
The picture shows how program calculates angles with noisy wall without filters

Change window size to activate mean filter (see upper)