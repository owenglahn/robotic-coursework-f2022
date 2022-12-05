Planner: Cubic polynomial

Implementation: I used a version of a state machine

World file: I was unable to create a world file. My VM would crash everytime I tried to save one and I still can't get ros to work properly on the Trottier machine I have access to.

How to run: roslaunch highlevel_controller a6.launch 

Target positions: Located in a6.yaml

Limitations: As mentioned before, no objects. 

Opening position: 0.0
Closing position: 0.7
Targets: I use a set of via points to set pre-grasp, grasp and release positions.
Result: It seems to work all the time. 