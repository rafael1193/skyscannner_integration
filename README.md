# SkyScanner integration

Software integration for the SkyScanner project

## Compilation

1. [Get ROS "Indigo"](http://wiki.ros.org/indigo/Installation) for your system following the instructions for *Desktop-Full Install*. (Newer versions should work but they haven't been tested) Don't forget to *initialize rosdep* and do the *environment setup*.
2. Install the following python dependencies (not exhaustive list):
    - numpy
    - scipy
    - matplotlib
    - netCDF4 (MesoNH integration)
3. Set up a SkyScanner ROS workspace:
   ```
   mkdir -p skyscanner_ws/src
   cd skyscanner_ws/src
   git clone https://github.com/rafael1193/skyscanner-integration.git
   cd ..
   echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```
4. Build the ROS package:
   ```
   catkin_make
   ```
5. Reload your bashrc and recompile again so ROS tools can detect the package properly:
   ```
   source ~/.bashrc
   catkin_make
   ```

Depending on the simulation backends you want to run you may have to install and configure some other dependencies:

 - **Paparazzi**
   - Get [Paparazzi UAV](http://wiki.paparazziuav.org/wiki/Main_Page) following the [instructions on their website](http://wiki.paparazziuav.org/wiki/Installation).
    - Set the environment variable `PAPARAZZI_SRC` with the folder where Paparazzi is located.
 - **FlightGear**
   - Install FlightGear from your package manager (version >3.0.0 should work)
   - Follow the instructions in [scripts/README.md](scripts/README.md).

## Execution

Beforehand, run `roscore` in a dedicated terminal window and leave it running.

*Launch* files define the nodes and parameters that have to be run to perform different tasks.

 - **Paparazzi**
   - Select your aircraft, *nps* target and build
   - Launch a *Simulation* session adding `-f127.0.0.1 -P50501` to the *Simulator* command line.`-P` should correspond with the port defined in `launch/conf/pprz_ac*.yaml` files and must be unique if several simulators are launched.
   - Run `roslaunch skyscanner pprz_planned.launch`
   - To command several paparazzi aircrafts go to the `pprz_planned.launch` file
 - **FlightGear**
   - Start FlightGear with the `fly_malolo1.sh` script. You can change the default UDP, Telnet and HTTP ports inside.
   - Run `roslaunch skyscanner fg_planned.launch` if you want to launch the whole architecture.
   - Run `roslaunch skyscanner fg_guided.launch` to start everything but the pathplanner so you can send tasks manually to the guidance node (by default it uses the VF alogrithm as guidance, but PLOS can be se too). In the following example `rostopic` is used to send to *ac_1* a *TrajectorySequence* composed of a circle of 200m radius and origin at (0, 0):
    ```
    rostopic pub /ac_1/trajectory_sequence skyscanner/TrajectorySequence "header:
     seq: 0
     stamp:
       secs: 0
       nsecs: 0
     frame_id: ''
    ac_id: 0
    trajectories:
    - circle: true
     origin: {x: 0.0, y: 0.0, z: 0.0}
     destination: {x: 0.0, y: 0.0, z: 0.0}
     radius: 200.0
     duration: 100.0
     time_limit: 10000000000.0" -1
    ```
 - Additionally `roslaunch skyscanner supervision.launch` will show online stats using *rqt_plot* and *rqt_plotxy* ([make sure you have them](https://github.com/rafael1193/rqt_common_plugins)).
 - **MesoNH**
   - By default realistic MesoNH wind is deactivated as you have to declare the path where the NetCDF files are stored in the parameter `mesonh_files_path` located in `launch/conf/mesonh.yaml`.
   - If wind data is available you can activate it switching `use_dummy_env` to `False` in `launch/conf/mesonh.yaml`.

Other configuration parameters can be tuned in `.yaml` files or directly in the roslaunch files you are running.
