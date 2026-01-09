# ros_system

This packages purpose is to keep the forklift in a healthy state and report any issues it cannot solve

### Node list
- **forklift_health**
    This node tracks hardware faults and reports them. This node should only run until it is done with it's health check after which it must be called again from a different node or the system must restart  
    TODO:
     - make method for a simple check and an extended background check
     - add logging in json format
- **system_health**  
    This node tracks the health of all the systems our software has control over (ros, can, remote op, etc.).
    The forklift_health node can be called upon from this file  
    TODO:
     - publish health data over /cavalier_health topic (one of the most important features will be displaying on the front end when the controls are off and allowing the remote operator to turn it on and off.
     - add loggin in json format for errors
- **teleop_start**
    This node starts everything necessary for teleop. This includes control scripts and driver aids. It also reads from the health topic to make sure it is in a good state.  
    TODO: 
     - add methods to start the system
     - first check system health over the health topic
     - fill in gaps in system_health (ex. if docker is not running, start it, if can is down, bring it back up)
     - fill in gaps in hardware_health (ex. if the drive is faulted, restart the forklift, if a sensor is down, try restarting it)
     - once the system is healthy, start the controller node and the driver aids
     - once all things have been started, stop the teleop start node
