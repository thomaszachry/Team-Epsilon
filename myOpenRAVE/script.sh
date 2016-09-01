#!/bin/bash
sudo apt-get -y install cmake g++ git ipython minizip python-dev python-h5py python-numpy python-scipy python-sympy qt4-dev-tools

sudo apt-get -y install libassimp-dev libavcodec-dev libavformat-dev libavformat-dev libboost-all-dev libboost-date-time-dev libbullet-dev libfaac-dev libglew-dev libgsm1-dev liblapack-dev liblog4cxx-dev libmpfr-dev libode-dev libogg-dev libpcrecpp0v5 libpcre3-dev libqhull-dev libqt4-dev libsoqt-dev-common libsoqt4-dev libswscale-dev libswscale-dev libvorbis-dev libx264-dev libxml2-dev libxvidcore-dev

mkdir myOpenRAVE && cd myOpenRAVE

git clone https://github.com/rdiankov/collada-dom.git
git clone --branch OpenSceneGraph-3.4 https://github.com/openscenegraph/OpenSceneGraph.git
git clone https://github.com/flexible-collision-library/fcl.git
git clone --branch latest_stable https://github.com/rdiankov/openrave.git

cd

cd myOpenRAVE && cd collada-dom && mkdir build && cd build
cmake ..
make -j4
sudo make install

cd

sudo apt-get -y install libcairo2-dev libjasper-dev libpoppler-glib-dev libsdl2-dev libtiff5-dev libxrandr-dev
cd myOpenRAVE && cd OpenSceneGraph && mkdir build && cd build
cmake .. -DDESIRED_QT_VERSION=4
make -j4
sudo make install

cd

sudo apt-get -y install libccd-dev
cd myOpenRAVE && cd fcl
git checkout 0.5.0
mkdir build && cd build
cmake ..
make -j4
sudo make install

cd

sudo ln -sf /usr/include/eigen3/Eigen /usr/include/Eigen

cd myOpenRAVE && cd openrave && mkdir build && cd build
cmake .. -DOSG_DIR=/usr/local/lib64/
make -j4
sudo make install

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(openrave-config --python-dir)/openravepy/_openravepy_
export PYTHONPATH=$PYTHONPATH:$(openrave-config --python-dir)

cd

echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(openrave-config --python-dir)/openravepy/_openravepy_" >> .bashrc
echo "export PYTHONPATH=$PYTHONPATH:$(openrave-config --python-dir)" >> .bashrc

openrave.py --example hanoi
