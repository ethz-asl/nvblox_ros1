ARG install_nvblox

FROM nvidia/cuda:11.8.0-devel-ubuntu20.04

# Install ROS first. This should be the only image where you have to
# manually install ROS so...
# Taken from OSRF docker file.

RUN apt-get -qq update && DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata

# We're in Zurich!
ENV TZ="Europe/Zurich"

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO noetic

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full

# Copy scripts folder
COPY scripts/ /root/scripts/
WORKDIR /root/
RUN chmod a+x -R /root/scripts

# Env variables
ENV NVBLOX_DEP_WS=/root/nvblox_dep_ws
ENV SCRIPTS_PATH=/root/scripts

# Run the general dep installation
RUN scripts/install_sys_deps.sh

# Run the ROS workspace set-up and dep installation
RUN scripts/install_ros_deps.sh

# Check out a copy of nvblox in there
RUN if [[ -z "$install_nvblox" ]] ; then scripts/install_nvblox.sh ; fi

# Finally, build all the stuff we downloaded.
RUN scripts/build_ros.sh
