# ==================================================================
# module list
# ------------------------------------------------------------------
# python        3.8    (apt)
# pytorch       latest (pip)
# ==================================================================

FROM nvidia/cudagl:11.1.1-devel-ubuntu20.04
ENV LANG C.UTF-8
ENV TZS=Europe/London
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone


# ==================================================================
# User setting
# ------------------------------------------------------------------

ARG USER
ARG PASSWD
ARG USER_ID

RUN apt-get update && \
    apt-get install -y sudo

ENV HOME /home/$USER

RUN useradd -m $USER -u $USER_ID && \
    gpasswd -a $USER sudo && \
    echo "$USER:$PASSWD" | chpasswd && \
    echo 'Defaults visiblepw' >> /etc/sudoers && \
    echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER $USER
WORKDIR $HOME

RUN APT_INSTALL="sudo apt-get install -y --no-install-recommends" && \
    PIP_INSTALL="python -m pip --no-cache-dir install --upgrade" && \
    GIT_CLONE="git clone --depth 10" && \

    sudo rm -rf /var/lib/apt/lists/* \
           /etc/apt/sources.list.d/cuda.list \
           /etc/apt/sources.list.d/nvidia-ml.list && \

    sudo apt-get update && \

# ==================================================================
# tools
# ------------------------------------------------------------------

    DEBIAN_FRONTEND=noninteractive $APT_INSTALL \
        build-essential \
        apt-utils \
        ca-certificates \
        wget \
        git \
        vim \
        libssl-dev \
        curl \
        unzip \
        unrar \
        zsh \
        neovim \
        tmux \
        && \

    $GIT_CLONE https://github.com/Kitware/CMake ~/cmake && \
    cd ~/cmake && \
    ./bootstrap && \
    sudo make -j"$(nproc)" install

# ==================================================================
# zsh
# ------------------------------------------------------------------
RUN APT_INSTALL="sudo apt-get install -y --no-install-recommends" && \
    PIP_INSTALL="python -m pip --no-cache-dir install --upgrade" && \
    GIT_CLONE="git clone --depth 10" && \

    zsh && \
    sudo chsh -s $(which zsh) && \

# ==================================================================
# python
# ------------------------------------------------------------------

    DEBIAN_FRONTEND=noninteractive $APT_INSTALL \
        software-properties-common \
        && \
    sudo add-apt-repository ppa:deadsnakes/ppa && \
    sudo apt-get update && \
    DEBIAN_FRONTEND=noninteractive $APT_INSTALL \
        python3-pip \
        python3.8 \
        python3.8-dev \
        python3-distutils-extra \
        && \
    wget -O ~/get-pip.py \
        https://bootstrap.pypa.io/get-pip.py && \
    sudo ln -s /usr/bin/python3 /usr/local/bin/python && \
    $PIP_INSTALL \
        numpy \
        scipy \
        pandas \
        cloudpickle \
        scikit-image>=0.14.2 \
        scikit-learn \
        matplotlib \
        Cython \
        tqdm \
        h5py \
        && \

# ==================================================================
# pytorch
# ------------------------------------------------------------------

    $PIP_INSTALL \
        future \
        numpy \
        protobuf \
        enum34 \
        pyyaml \
        typing \
        && \
    $PIP_INSTALL \
        torch \
        torchvision \
        torchaudio -f \
        https://download.pytorch.org/whl/cu112/torch_stable.html \
        && \

# ==================================================================
# zsh
# ------------------------------------------------------------------

    sh -c "$(wget -O- https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" && \
    $GIT_CLONE https://github.com/junjungoal/dotfiles.git && \
    sh dotfiles/dotfilesLink.sh && \
    $GIT_CLONE https://github.com/zsh-users/zsh-autosuggestions ~/.zsh/zsh-autosuggestions && \
    $(which zsh) -c "source ~/.zsh/zsh-autosuggestions/zsh-autosuggestions.zsh" && \
    $(which zsh) -c "source ~/.zshrc" && \
    rm -rf ~/.vim && \
    $GIT_CLONE https://github.com/VundleVim/Vundle.vim.git ~/.vim/bundle/Vundle.vim && \
    mkdir .config && \
    cp -r dotfiles/config/* .config/ && \


# ==================================================================
# MUJOCO
# ------------------------------------------------------------------
    mkdir -p $HOME/.mujoco \
        && wget https://mujoco.org/download/mujoco210-linux-x86_64.tar.gz -O mujoco.tar.gz \
        && tar -xvf mujoco.tar.gz \
        && mv mujoco210 $HOME/.mujoco/ \
        && rm mujoco.tar.gz

ENV LD_LIBRARY_PATH $HOME/.mujoco/mujoco210/bin:${LD_LIBRARY_PATH}

ENV LD_LIBRARY_PATH /usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

RUN PIP_INSTALL="python -m pip --no-cache-dir install --upgrade" && \

    $PIP_INSTALL \
    mujoco-py \
    gym

# ===================================================================
# Install ROS noetic dependencies.
# ===================================================================

RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" | sudo tee /etc/apt/sources.list.d/ros-latest.list
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

RUN APT_INSTALL="sudo apt-get install -y --no-install-recommends" && \
    sudo apt-get update && \
    DEBIAN_FRONTEND=noninteractive $APT_INSTALL \
        pkg-config \
        libxau-dev \
        libxdmcp-dev \
        libxcb1-dev \
        libxext-dev \
        libx11-dev \
        git-core \
        python-argparse \
        python3-rosdep \
        python3-rosinstall \
        python3-vcstools \
        python3-wstool \
        ros-noetic-desktop-full \
        ros-noetic-actionlib \
        ros-noetic-actionlib-msgs \
        ros-noetic-control-msgs \
        ros-noetic-cv-bridge \
        ros-noetic-rospy-message-converter \
        ros-noetic-rviz \
        ros-noetic-tf2-ros \
        ros-noetic-trajectory-msgs \
        ros-noetic-xacro \
        ros-noetic-moveit \
        ros-noetic-mocap-optitrack \
        ros-noetic-joy \
        ros-noetic-combined-robot-hw \
        ros-noetic-dynamic-reconfigure \
        build-essential \
        cmake \
        libpoco-dev \
        libeigen3-dev \
        dbus-x11 && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*


RUN \
    sudo rosdep init && \
    rosdep update

RUN mkdir -p ~/catkin_ws/src

RUN ["/bin/zsh", "-c", \
    "source /opt/ros/noetic/setup.zsh && \
    cd ~/catkin_ws && \
    catkin_make"]

# =========================================================================================
# Install Franka
# =========================================================================================

RUN pip install numpy-quaternion

# # Download and build libfranka
RUN ["/bin/zsh", "-c", \
    "git clone --recursive https://github.com/frankaemika/libfranka $HOME/libfranka && \
    mkdir $HOME/libfranka/build && \
    cd $HOME/libfranka/build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    cmake --build ."]

RUN ["/bin/zsh", "-c", \
  "cd ~/catkin_ws/src && \
  git clone --recursive https://github.com/frankaemika/franka_ros && \
  git clone https://github.com/junjungoal/panda_robot.git && \
  git clone https://github.com/ros-planning/panda_moveit_config.git && \
  cd ~/catkin_ws/src/franka_ros && \
  git checkout 0.7.1 && \
  cd ~/catkin_ws/ && \
  rosdep install --from-paths src --ignore-src --rosdistro noetic -r -y --skip-keys libfranka && \
  source /opt/ros/noetic/setup.zsh && \
  catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=$HOME/libfranka/build && \
  source ~/catkin_ws/devel/setup.zsh && \
  echo 'source /opt/ros/noetic/setup.zsh' >> ~/.zshrc && \
  echo 'source ~/catkin_ws/devel/setup.zsh' >> ~/.zshrc"]

# =========================================================================================
# Install Realsense
# =========================================================================================
RUN sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

RUN APT_INSTALL="sudo apt-get install -y --no-install-recommends" && \
    sudo apt-get update && \
    DEBIAN_FRONTEND=noninteractive $APT_INSTALL \
        librealsense2-dkms \
        librealsense2-utils \
        librealsense2-dev \
        librealsense2-dbg \
        ros-noetic-realsense2-camera \
	swig

# git clone https://github.com/pal-robotics/ddynamic_reconfigure.git && \
# git clone https://github.com/ros/dynamic_reconfigure.git && \
RUN ["/bin/zsh", "-c", \
  "cd ~/catkin_ws/src && \
   git clone https://github.com/IntelRealSense/realsense-ros.git && \
   git clone https://github.com/usrl-uofsc/stag_ros.git && \
   cd ~/catkin_ws/ && \
   source ~/.zshrc && \
   catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_OPENMP=OFF && \
   catkin_make install --pkg realsense2_camera && \
   source ~/catkin_ws/devel/setup.zsh"]

RUN ["/bin/zsh", "-c", \
  "cd ~/ && \
   git clone https://github.com/stevengj/nlopt.git && \
   cd ~/nlopt && \
   mkdir build && \
   cd ~/nlopt/build && \
   cmake .. && \
   make && \
   sudo make install"]

RUN ["/bin/zsh", "-c", \
  "cd ~/catkin_ws/src && \
   git clone -b devel https://clemi@bitbucket.org/clemi/trac_ik.git && \
   cd ~/catkin_ws/ && \
   source ~/.zshrc && \
   catkin_make -DBUILD_WITH_OPENMP=OFF  -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/home/jyamada/libfranka/build -DCATKIN_BLACKLIST_PACKAGE='trac_ik;trac_ik_examples' && \
   source ~/catkin_ws/devel/setup.zsh"]

RUN pip install wandb sympy

RUN echo "export ROS_MASTER_URI=http://bamtoo:11311" >> ~/.zshrc
RUN echo "export ROS_IP=192.168.4.3" >> ~/.zshrc

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
ENV LIBGL_ALWAYS_INDIRECT 0


RUN sudo apt-get update && \
    sudo apt-get -y install \
    libosmesa6-dev \
    patchelf \
    ros-noetic-realsense2-camera \
    ros-noetic-realsense2-description

Run pip install tensorboardX \
                moviepy \
                imageio \
                funcsigs
RUN ["/bin/zsh", "-c", \
 "nvim +VundleInstall +qall"]
# ==================================================================
# config & cleanup
# ------------------------------------------------------------------

RUN mkdir -p ~/projects

RUN sudo ldconfig && \
    sudo apt-get clean && \
    sudo apt-get autoremove && \
    sudo rm -rf /var/lib/apt/lists/* /tmp/*
