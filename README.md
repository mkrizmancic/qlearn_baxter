[english version](#baxter-robot-manipulative-tasks-using-machine-learning)

# Izvedba manipulacijskih zadataka Baxter robota primjenom strojnog učenja
Za poznati problem rješavanja hanojskih tornjeva u najmanjem broju koraka razvijen je algoritam upravljanja zasnovan na tehnici podržanog strojnog učenja Q-Learning. Nakon simulacije na računalu, algoritam je potvrđen eksperimentalno na robotu Baxter. Robot uz pomoć OptiTrack sustava lokalizira predmete u radnom prostoru te transformira njihove lokacije u vlastiti koordinatni sustav. Zatim pomoću već razvijenog modela upravljanja izvršava slijed radnji potrebnih za premještanje kolutova. Upravljanje robotom te međusobna komunikacija između računala, robota i OptiTrack sustava ostvareni su korištenjem ROS-a.

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


### Upute za pokretanje
U datoteci [across.launch](../launch/across.launch) po potrebi promijeniti adresu servera.

Prilikom svakog otvaranja nove konzole:
```
roscd
cd ..
.\baxter.sh
```

**Za pronalažanje transformacija:**
- u prvu konzolu: ```roslaunch qlearn_baxter find_transformations.launch```
- u drugu konzolu: ```rosrun qlearn_baxter interpolate.py```
- pogledati upute u docstringu u ```interpolate.py```


**Za testiranje:**
- ```roslaunch qlearn_baxter testing.launch```

**Za izvedbu zadatka:**
- ```roslaunch qlearn_baxter main.launch```

**Za pokretanje samo Q-Learning algoritma:**
- ```python QLearning.py```


# Baxter robot manipulative tasks using machine learning
In order to solve a well-known tower of Hanoi problem, an algorithm based on reinforcement machine learning technique Q-learning is developed. After simulation on computer, the algorithm is experimentally confirmed using Baxter robot. Using OptiTrack localization system, the robot finds objects in its working area and transforms their locations in its own coordinate system. Then, using already implemented control model, it performs a set of moves necessary to solve the problem. Control of the robot, as well as the communication between computer, robot and OptiTrack system are implemented using ROS.

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

### Execution
If needed, change the server adress in [across.launch](../launch/across.launch).

**Finding transformations:**
- in first console: ```roslaunch qlearn_baxter find_transformations.launch```
- in second console: ```rosrun qlearn_baxter interpolate.py```
- follow the instructions given in docstring of ```interpolate.py```

**Testing:**
- ```roslaunch qlearn_baxter testing.launch```

**Starting complete system:**
- ```roslaunch qlearn_baxter main.launch```

**Only Q-Learning algorithm**
- ```python QLearning.py```

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
