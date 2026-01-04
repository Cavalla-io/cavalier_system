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
     - literally everything lol