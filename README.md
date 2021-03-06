# kuka-lpvds-tasks
This package implements the LPV-DS motion generator from [1] together with the passive-DS controller for the KUKA LWR 4+ robot in both simulation (gazebo) and with the real robot for the following tasks **learned from demonstration**:

- Task 1: Inspection Line  
<p align="center">
<img src="https://github.com/epfl-lasa/kuka-lpvds-tasks/blob/master/img/inspection-demo.gif"  width="350"></>
  <img src="https://github.com/epfl-lasa/kuka-lpvds-tasks/blob/master/img/inspection-exec.gif"  width="350"></>

- Task 2: Branding Line
<p align="center">
  <img src="https://github.com/epfl-lasa/kuka-lpvds-tasks/blob/master/img/branding-demo.gif"  width="350"></>   
<img src="https://github.com/epfl-lasa/kuka-lpvds-tasks/blob/master/img/branding-exec.gif" width="350"></>

- Task 3: Shelf-Arranging (top only)
<p align="center">
  <img src="https://github.com/epfl-lasa/kuka-lpvds-tasks/blob/master/img/shelf-demo.gif" width="350"></>   
<img src="https://github.com/epfl-lasa/kuka-lpvds-tasks/blob/master/img/shelf-exec.gif" width="350"></>

Videos of the execution of these tasks on the real robot can be found here: [robot-experiments](http://lasa.epfl.ch/files/Nadia/Figueroa-CoRL2018-Experiments.mp4)

- Task 4: Letter Writing (from LASA-Dataset)


### Dependencies
To run this package you must install the following dependencies:
- [kuka-lwr-ros](https://github.com/epfl-lasa/kuka-lwr-ros.git) ``checkout 'nadia' branch`` | Control Interface for Kuka LWR 4+
- [ds-motion-generator](https://github.com/epfl-lasa/ds_motion_generator.git) ``checkout 'nadia' branch`` | DS motion generation nodes
- [lpvDS-lib](https://github.com/nbfigueroa/lpvDS-lib) | lpv-DS class used by ds-motion-generator
- [grasp-interface](https://github.com/epfl-lasa/grasp_interface) | To control the Robotiq S gripper from code 

### Simulation
1. Bring up the kuka-lwr-ros controller and simulator:
```
$ roslaunch kuka_lpvds_tasks sim.launch force-interface:=true
```
2. Run the planning-interface to send joint commands:
```
$ roslaunch kuka_lpvds_tasks planning_client.launch
$ roslaunch kuka_lpvds_tasks planning_console.launch
```
Default commands (go_home, go_left, go_right, go_candle)
These commands are use to send the robot to a "good" initial joint configuration, generally you should ```go_home``` or ```go_right``` to test a learned ds-motion generator.

3. Load DS motion generators and Task Planning Node  
- For Inspection Line Task
  ```
  $ roslaunch kuka_lpvds_tasks run_inspection_task.launch sim:=true
  ```
- For Branding Line Task
  ```
  $ roslaunch kuka_lpvds_tasks run_branding_task.launch sim:=true
  ```
- For Shelf Arranging Task
  ```
  $ roslaunch kuka_lpvds_tasks run_shelf_task.launch sim:=true
  ```

To apply external forces during the execution (to test the reactivity of the DS and impedance controller) you can define the force in the following topic:
```
/lwr/joint_controllers/passive_ds_external_force
```
the force will be applied by manipulating the boolean command:
```
/lwr/joint_controllers/passive_ds_apply_force
```

### Real robot
To run the tasks on the real robot you should follow the same instructions above, except for
1. Bring up the kuka-lwr-ros controller and console in different terminals: 
```
$ roslaunch lwr_simple_example real.launch
$ roslaunch lwr_fri lwr_fri_console.launch
```
and for the **3. Load DS motion generators and Task Planning Node** you should either set ``sim:=false`` or not set at all, as it is the default.

Additionally you should bring up the gripper grasp-interface:
```
$ roslaunch grasp_interface rs_gripper.launch
```
To modify the gripper state during execution you can launch the gripper voice controller from the [demo-voice-control](https://github.com/epfl-lasa/demo-voice-control) package:
```
roslaunch demo_voice_control gripper_voice_control.launch
```

**References**   
> [1] Figueroa, N. and Billard, A. (2018) "A Physically-Consistent Bayesian Non-Parametric Mixture Model for Dynamical System Learning". Conference on Robot Learning (CoRL) - 2018 Edition. To Appear. 

**Contact**: [Nadia Figueroa](http://lasa.epfl.ch/people/member.php?SCIPER=238387) (nadia.figueroafernandez AT epfl dot ch)

**Acknowledgments**
This work was supported by the EU project [Cogimon](https://cogimon.eu/cognitive-interaction-motion-cogimon) H2020-ICT-23-2014.
