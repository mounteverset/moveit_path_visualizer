FROM moveit/moveit:noetic-ci-shadow-fixed
#Umut

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*
    
RUN cd /opt/ros/noetic/share && \ 
	apt-get update && apt-get install -y ros-noetic-rqt && \
	apt-get install -y ros-noetic-rqt-common-plugins && \
	apt-get install -y ros-noetic-rqt-moveit && \
	apt-get install -y ros-noetic-rqt-robot-plugins
	
    
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc



RUN apt-get install -y ros-noetic-catkin python3-catkin-tools
RUN apt-get install -y python3-wstool

RUN apt-get -qq update && \
    apt-get -qq dist-upgrade
	
# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
# https://docs.docker.com/develop/develop-images/dockerfile_best-practices/#minimize-the-number-of-layers
ENV ROS_UNDERLAY /root/ws_moveit/install
WORKDIR $ROS_UNDERLAY/../src
    
RUN \
    # Download moveit source so that we can get necessary dependencies
    wstool init . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall && \
    wstool update -t .
    #
    # Update apt package list as cache is cleared in previous container
    # Usually upgrading involves a few packages only (if container builds became out-of-sync)
     
RUN cd ~/ws_moveit/src
RUN rosdep update && \
    rosdep install -y --from-paths . --ignore-src --rosdistro noetic && \
    rm -rf /var/lib/apt/lists/*
    
RUN cd .. && \
    git clone https://github.com/ros-industrial/industrial_core.git src/industrial_core && \
    git clone -b kinetic https://github.com/ros-industrial/fanuc.git src/fanuc && \
    rm -rf src/industrial_core/industrial_trajectory_filters



ENV PYTHONIOENCODING UTF-8
WORKDIR /root/ws_moveit
RUN catkin config --extend /opt/ros/$ROS_DISTRO --isolate-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    # Status rate is limited so that just enough info is shown to keep Docker from timing out,
    # but not too much such that the Docker log gets too long (another form of timeout)
    catkin build --limit-status-rate 0.001 --no-notify

RUN echo "source /root/ws_moveit/devel/setup.bash" >> ~/.bashrc