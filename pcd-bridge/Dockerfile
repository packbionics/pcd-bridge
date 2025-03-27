FROM ros:noetic-ros-core

WORKDIR /workspace

RUN apt-get update
RUN apt-get install python3-rosdep -y

RUN rosdep init
RUN rosdep update

# RUN rosdep install -y --from-paths src --rosdistro noetic

RUN apt-get install build-essential python3-pip -y

RUN pip install paho-mqtt

COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT /entrypoint.sh