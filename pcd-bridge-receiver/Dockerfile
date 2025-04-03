FROM ros:humble-ros-core

WORKDIR /workspace

RUN apt-get update
RUN apt-get install python3-rosdep -y

RUN rosdep init
RUN rosdep update

COPY . .

RUN rosdep install -y --from-paths src --rosdistro humble

RUN apt-get install build-essential python3-pip -y

RUN pip install paho-mqtt

RUN apt-get update && apt-get install python3-colcon-common-extensions -y

RUN apt-get install ros-humble-rmw-cyclonedds-cpp -y
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# RUN source /opt/ros/humble/setup.bash && colcon build

RUN chmod +x /workspace/entrypoint.sh

ENTRYPOINT ["/workspace/entrypoint.sh"]
CMD ["pcd_bridge_receiver"]