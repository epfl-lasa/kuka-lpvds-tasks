# kuka-lpvds-compliant


## DEPENDENCIES


## SIMULATION

```
$ roslaunch lwr_simple_example sim.launch force-interface:=true
```
To apply external forces during the execution of the passive-DS controller you can define the force in the following topicL
```
/lwr/joint_controllers/passive_ds_external_force
```
the force will be applied by manipulating the boolean command:
```
/lwr/joint_controllers/passive_ds_apply_force
```

## On REAL ROBOT
