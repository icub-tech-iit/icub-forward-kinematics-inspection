FROM ubuntu:latest
LABEL org.opencontainers.image.title="iCub Forward Kinematics Inspection"
LABEL org.opencontainers.image.description="Infrastructure for running tests on the kinematics of of an iCub limb"
LABEL org.opencontainers.image.source="https://github.com/icub-tech-iit/icub-forward-kinematics-inspection"
LABEL org.opencontainers.image.authors="Mattia Fussi <mattia.fussi@iit.it>"

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive

# Update apt database
RUN apt update

# Install essentials
RUN apt install -y apt-utils software-properties-common apt-transport-https sudo \
    psmisc lsb-release tmux nano wget curl gedit build-essential gdb git gitk \
    cmake cmake-curses-gui autoconf locales gdebi terminator meld dos2unix bash-completion

# Set the locale
RUN locale-gen en_US.UTF-8

# Install graphics
RUN apt install -y xfce4 xfce4-goodies xserver-xorg-video-dummy xserver-xorg-legacy x11vnc firefox && \
    sed -i 's/allowed_users=console/allowed_users=anybody/' /etc/X11/Xwrapper.config
COPY xorg.conf /etc/X11/xorg.conf
RUN dos2unix /etc/X11/xorg.conf

# Install python
RUN apt install -y python3 python3-dev python3-pip python3-setuptools && \
    if [ ! -f "/usr/bin/python" ]; then ln -s /usr/bin/python3 /usr/bin/python; fi

# Install noVNC
RUN git clone https://github.com/novnc/noVNC.git /opt/novnc && \
    git clone https://github.com/novnc/websockify /opt/novnc/utils/websockify && \
    echo "<html><head><meta http-equiv=\"Refresh\" content=\"0; url=vnc.html?autoconnect=true&reconnect=true&reconnect_delay=1000&resize=scale&quality=9\"></head></html>" > /opt/novnc/index.html

# Install dependencies
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    apt update && \
    apt install -y libeigen3-dev coinor-libipopt-dev freeglut3-dev \
    libboost-system-dev libboost-filesystem-dev libboost-thread-dev libtinyxml-dev libace-dev libedit-dev \
    libgsl0-dev libopencv-dev libode-dev liblua5.1-dev lua5.1 swig qtbase5-dev qtdeclarative5-dev \
    qtmultimedia5-dev qml-module-qtquick2 qml-module-qtquick-window2 qml-module-qtmultimedia \
    qml-module-qtquick-dialogs qml-module-qtquick-controls qml-module-qt-labs-folderlistmodel \
    qml-module-qt-labs-settings libsdl1.2-dev libxml2-dev libv4l-dev \
    libjpeg-dev libmatio-dev libirrlicht-dev libspdlog-dev \
    gazebo11 libgazebo11-dev

# Select options
ARG ROBOTOLOGY_SUPERBUILD_RELEASE
ARG ICUB_MAIN_FORK
ARG BUILD_TYPE=Release
ARG ROBOTOLOGY_SUPERBUILD_INSTALL_DIR=/usr/local

# Set up git (required by superbuild)
RUN git config --global user.name "GitHub Actions" && \
    git config --global user.email "actions@github.com"

# Build robotology-superbuild w/ the specified fork of icub-main
RUN git clone https://github.com/robotology/robotology-superbuild.git --depth 1 --branch ${ROBOTOLOGY_SUPERBUILD_RELEASE} && \
    cd robotology-superbuild && mkdir build && cd build && \
    cmake .. \
          -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
          -DYCM_EP_INSTALL_DIR=${ROBOTOLOGY_SUPERBUILD_INSTALL_DIR} \
          -DROBOTOLOGY_ENABLE_CORE:BOOL=ON \
          -DROBOTOLOGY_ENABLE_DYNAMICS:BOOL=ON \
          -DROBOTOLOGY_ENABLE_ICUB_BASIC_DEMOS:BOOL=ON \
          -DROBOTOLOGY_ENABLE_ROBOT_TESTING:BOOL=ON \
          -DICUB_TAG=master \
          -DICUB_REPOSITORY=${ICUB_MAIN_FORK}.git && \
          make

# Clean up git configuration
RUN git config --global --unset-all user.name && \
    git config --global --unset-all user.email

# Build package containing forward kinematics inspection
RUN git clone https://github.com/icub-tech-iit/icub-forward-kinematics-inspection.git && \
    cd icub-forward-kinematics-inspection && mkdir build && cd build && \
    /bin/bash -c 'source ${ROBOTOLOGY_SUPERBUILD_INSTALL_DIR}/share/robotology-superbuild/setup.sh' && \
    cmake .. -DCMAKE_BUILD_TYPE=${BUILD_TYPE} && \
    make install

# Set environmental variables
ENV DISPLAY=:1

# Set up .bashrc
RUN echo "source /etc/profile.d/bash_completion.sh" >> /root/.bashrc && \
    echo "source ${ROBOTOLOGY_SUPERBUILD_INSTALL_DIR}/share/robotology-superbuild/setup.sh" >> /root/.bashrc

COPY cartesianSolver.ini /usr/local/share/iCub/contexts/simCartesianControl
COPY left_arm_cartesian.xml /usr/local/share/iCub/contexts/simCartesianControl/cartesian
COPY right_arm_cartesian.xml /usr/local/share/iCub/contexts/simCartesianControl/cartesian

# Set up script to launch graphics and vnc
COPY start-vnc-session.sh /usr/bin/start-vnc-session.sh
RUN chmod +x /usr/bin/start-vnc-session.sh && \
    dos2unix /usr/bin/start-vnc-session.sh

# Manage ports
EXPOSE 5901 6080 10000/tcp 10000/udp

# Clean up unnecessary installation products
RUN rm -Rf /var/lib/apt/lists/*

# Launch bash from /root
WORKDIR /root
CMD ["bash"]
