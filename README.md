# pcd-bridge

## Introduction

This repository contains software configured to interface ROS 1 point cloud data publishers with ROS 2 point cloud data subscribers. The code in this repository is intended to replace the use of [ros1_bridge](https://github.com/ros2/ros1_bridge). `ros1_bridge` is useful for generally interfacing ROS 1 and ROS 2 software, but is associated with issues when used within Docker. This software uses the MQTT protocol to transfer data between ROS 1 and ROS 2 applications.

## Prerequisites

* [Docker](https://www.docker.com/)

## How to Install

Clone the repository to your local machine

```bash
git clone git@github.com:packbionics/pcd-bridge.git
```

## How to Use

Run the `docker-compose.yml` file at the root of the repository

```bash
docker compose up -d
```
**Note:** If you do not wish to run the container in detached mode, run the above command without `-d`
