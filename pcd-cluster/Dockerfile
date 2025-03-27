FROM osrf/ros:kinetic-desktop

RUN apt-get update && apt-get install ros-kinetic-perception-pcl -y

# ENV ROS_MASTER_URI=http://pcd-cluster:11311/
# ENV ROS_HOSTNAME=pcd-cluster


CMD [ "roslaunch", "pcl_ros", "sample_extract_clusters.launch" ]