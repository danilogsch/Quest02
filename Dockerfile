ARG ROS_DISTRO=humble
 
########################################
# Base Image for TurtleBot3 Simulation #
########################################
FROM osrf/ros:${ROS_DISTRO}-desktop as base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]
 
# # Create Colcon workspace
# RUN mkdir -p /ros2_ws/src
 
# # Build the base Colcon workspace, installing dependencies first.
# WORKDIR /ros2_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y 

 # Use Cyclone DDS as middleware
 RUN apt-get update && apt-get install -y --no-install-recommends \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
 ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

 FROM base as dev
 
 # Dev container arguments
 ARG USERNAME=devuser
 ARG UID=1000
 ARG GID=${UID}
  
 # Install extra tools for development
 RUN apt-get update && apt-get install -y --no-install-recommends \
  gdb gdbserver nano
  
 # Create new user and home directory
 RUN groupadd --gid $GID $USERNAME \
  && useradd --uid ${GID} --gid ${UID} --create-home ${USERNAME} \
  && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
  && chmod 0440 /etc/sudoers.d/${USERNAME} \
  && mkdir -p /home/${USERNAME} \
  && chown -R ${UID}:${GID} /home/${USERNAME}
  
 # Set the ownership of the ros2 workspace to the new user
#  RUN chown -R ${UID}:${GID} /ros2_ws/"
  
 # Set the user and source entrypoint in the user's .bashrc file
 USER ${USERNAME}
#  RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc
 RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc
 WORKDIR /home/${USERNAME}
 # Create Colcon workspace with external dependencies
 RUN mkdir -p /home/${USERNAME:-devuser}/ros2_ws/src
  
 # Build the base Colcon workspace, installing dependencies first.
 WORKDIR /home/${USERNAME}/ros2_ws
 RUN colcon build --symlink-install
