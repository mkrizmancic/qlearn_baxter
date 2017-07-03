[english version](#baxter-robot-manipulative-tasks-using-machine-learning)

# Izvedba manipulacijskih zadataka Baxter robota primjenom strojnog učenja

Iako je ovaj paket namijenjen korištenju uz Baxter robota, implementacija Q-learning algoritma se može koristiti nezavisno.

### Upute za instalaciju:
**Napomena:** Svugdje gdje se traži ROS verzija koristiti 'kinetic' (često umjesto 'indigo')
1. Instalirati ROS (Kinetic): http://wiki.ros.org/kinetic/Installation/Ubuntu
2. Instalirati Baxter SDK prema [uputama](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup#Step_3:_Create_Baxter_Development_Workspace)
3. Instalirati MoveIt! za ROS Kinetic prema [uputama](http://moveit.ros.org/install/)
4. Instalirati VRPN prema [uputama](#upute-za-instalaciju-vrpn-a)
5. Instalirati ovaj paket sljedećim naredbama:
```
roscd
cd ../src
git clone https://github.com/mkrizmancic/qlearn_baxter.git
cd ..
catkin_make
```


### Postavljanje okruženja


# Baxter robot manipulative tasks using machine learning

Although this package is intended to be used with Baxter robot, Q-Learning algorithm is stand-alone.

### Installation:
**Note:** Everywhere ROS version is asked, use 'kinetic' (often instead of 'indigo')
1. Install ROS (Kinetic): http://wiki.ros.org/kinetic/Installation/Ubuntu
2. Install Baxter SDK following this [instructions](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup#Step_3:_Create_Baxter_Development_Workspace)
3. Install MoveIt! for ROS Kinetic following this [instructions](http://moveit.ros.org/install/)
4. Install VRPN following this [instructions](#upute-za-instalaciju-vrpn-a)
5. Install this package with follwing commands:
```
roscd
cd ../src
git clone https://github.com/mkrizmancic/qlearn_baxter.git
cd ..
catkin_make
```
### Environment setup



### Upute za instalaciju VRPN-a / Instructions for installing VRPN
```
cd ~
mkdir CustomPackages
cd CustomPackages
git clone https://github.com/vrpn/vrpn.git
cd vrpn
mkdir build
cd build
cmake ..
make
sudo make install
roscd
cd ../src
git clone https://github.com/larics/vrpn_client_ros.git
cd ..
catkin_make
```
