# kuka-lpvds-tasks
This package implements the LPV-DS motion generator from [1] together with the passive-DS controller for the KUKA LWR 4+ robot in both simulation (gazebo) and with the real robot for the following tasks learned from demonstration:

- Scenario 1: Production Line
- Scenario 2: Inspection Line
- Scenario 3: Shelf-Arranging top and bottom
- Scenario 4: Cube Picking
- Scenario 5: Bumpy Surface Drawing

### Dependencies
To run this package you must install the following dependencies:
- [kuka-lwr-ros](https://github.com/epfl-lasa/kuka-lwr-ros.git) checkout 'nadia' branch
- [ds-motion-generator](https://github.com/epfl-lasa/ds_motion_generator.git) checkout 'nadia' branch

### Simulation
1. Bring up the kuka-lwr-ros controller and simulator:
```
$ roslaunch kuka_lpvds_compliant sim.launch force-interface:=true
```
2. Run the planning-interface to send joint commands:
```
$ roslaunch kuka_lpvds_compliant planning_client.launch
$ roslaunch kuka_lpvds_compliant planning_console.launch
```
Default commands (go_home, go_left, go_right, go_candle)
These commands are use to send the robot to a "good" initial joint configuration, generally you should ```go_home``` or ```go_right``` to test a learned ds-motion generator.

3. Load DS motion generator **(TODO: Change to LPV-DS)**:
```
$ roslaunch ds_motion_generator load_DS_motionGenerator.launch
```
4. To apply external forces during the execution of the passive-DS controller you can define the force in the following topic:
```
/lwr/joint_controllers/passive_ds_external_force
```
the force will be applied by manipulating the boolean command:
```
/lwr/joint_controllers/passive_ds_apply_force
```

### Real robot
1. Bring up the kuka-lwr-ros controller and console in different terminals: 
```
$ roslaunch roslaunch kuka_lpvds_compliant real.launch
$ roslaunch lwr_fri lwr_fri_console.launch
```
2. Run the planning-interface to send joint commands:
```
$ roslaunch kuka_lpvds_compliant planning_client.launch
$ roslaunch kuka_lpvds_compliant planning_console.launch
```
Default commands (go_home, go_left, go_right, go_candle)
These commands are use to send the robot to a "good" initial joint configuration, generally you should ```go_home``` or ```go_right``` to test a ds-motion generator. If the robot is currently in a weird, near collision configuration ```go_candle``` first.

3. ...


### Reference
[1] Figueroa, N. and Billard, A. (2018) "A Physically-Consistent Bayesian Non-Parametric Mixture Model for Dynamical System Learning". Conference on Robot Learning (CoRL) - 2018 Edition. Accepted. 
