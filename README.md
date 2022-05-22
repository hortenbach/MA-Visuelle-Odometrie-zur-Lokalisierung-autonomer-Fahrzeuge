# MA-Visuelle-Odometrie-zur-Lokalisierung-autonomer-Fahrzeuge

Visual odometry for localization of autonomous vehicles with CARLA Simulator and ROS2 Nodes. 

Run 

```
$ git submodule init
```

to fetch submodules in /src folder.

see src/start.py for the setup.
Launch viusal odometry node with thesis setting via launch scripts


```
$ ros2 launch src/carla_vo_stereo_launch.py

$ ros2 launch src/carla_vo_rgbd_launch.py
```

See src/py folder for helper scripts and Notebooks with prototyping and testing like /src/py/VisualOdometryMA.ipynb
