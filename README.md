# map3d_server

this package is use to read binary map, and publish pointclouds/girdmap to ros system.  

## install:

### Build and install PCL 1.8.
sudo apt -y install libflann1.8 libboost1.58-all-dev  
cd ~/Downloads  
wget http://launchpadlibrarian.net/209530212/libeigen3-dev_3.2.5-4_all.deb  
sudo dpkg -i libeigen3-dev_3.2.5-4_all.deb  
sudo apt-mark hold libeigen3-dev  

wget http://www.vtk.org/files/release/7.1/VTK-7.1.0.tar.gz  
tar -xf VTK-7.1.0.tar.gz  
cd VTK-7.1.0 && mkdir build && cd build  
cmake ..  
make                                                               
sudo make install  

cd ~/Downloads  
wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.0.tar.gz  
tar -xf pcl-1.8.0.tar.gz  
cd pcl-pcl-1.8.0 && mkdir build && cd build  
cmake ..  
make  
sudo make install  

cd ~/Downloads  
rm libeigen3-dev_3.2.5-4_all.deb VTK-7.1.0.tar.gz pcl-1.8.0.tar.gz  
sudo rm -r VTK-7.1.0 pcl-pcl-1.8.0  

### Build and install map3d_server.    
cd catkin_ws/src  
git clone http://padflexc12.vserv.jp.panasonic.com:8080/gitbucket/git/ATMOB/map3d_server.git
cd ..
catkin_make


## usage:
roscd map3d_server
gedit launch/map3d_server.launch
    <node name="map3d_server" pkg="map3d_server" type="map3d_server" respawn="false" output="screen" >  
    <param name="binary_filename"    type="string" value= "/home/liu/catkin_ws/src/map3d_server/map/gazebo.pcd" />  <--you binary map
    <param name="map3d_topic_name"   type="string" value= "/map3d" />  
    <param name="map2d_topic_name"   type="string" value= "/map" />  
    <param name="grid_size"          type="double"  value= "0.05" />  
    <param name="min_z"              type="double"  value= "-0.2" />  
    <param name="max_z"              type="double"  value= "1" />  
    <param name="min_prob"           type="double"  value= "0.53" />  
roslaunch map3d_server map3d_server.launch




# mesh
