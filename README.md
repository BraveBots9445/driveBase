# README
This is a basic template to get our drivebase running with pathplanner and vision.

# Setup
To set up swerve for your usecase, you will modify the tuner-project.json and generated/tuner_constants.py files. These can be generated from the Phoenix6 Tuner in Phoenix Tuner X. (See the [docs](https://v6.docs.ctr-electronics.com/en/latest/docs/tuner/tuner-swerve/index.html))
To set up vision for your usecase, you can modify subsystems/vision.py. Most likely, the positions of the cameras defined in the subsystem constructor will be a majority of your modifications. You may also change the base standard deviations found above the constructor.  (See the [docs](https://docs.photonvision.org/en/latest/index.html))
To set up pathplanner, you can 
1. Open this project in pathplanner itself and create paths. 
2. Set any needed named commands in RobotContainer.py in the ```set_pp_named_commands``` function
3. Tune the PathPlanner constants in subsystems/drivetrain.py around line 267.

# Running
Any below commands can be run in VSCode by pressing ctrl+\` to get a terminal and typing or copying them as shown.
In your terminal, you should be at the root directory of the project. Your terminal prompt may look something like:
```
PS C:\Users\bryso\Desktop\Robotics\driveBase>
```
Each machine can have different ways that python is installed. In the below examples, ```robotpy``` is assumed to be on the path. If the commands fail because robotpy is not found, try a few alternatives:
```
robotpy {the rest of the command}
python3 -m robotpy {the rest of the command}
py3 -m robotpy {the rest of the command}
py -m robotpy {the rest of the command}
```
If none of those work, there is a chance that you do not have robotpy installed. Consult the [docs](https://robotpy.readthedocs.io/en/stable/)

You should have the proper versions of packages installed. To ensure this, run 
```robotpy sync```
You may be prompted to increase the version of robotpy. Only do this if you understand the consequences and/or are directed to by a team leader or mentor. 
## Simulation
To run simulation, run 

```robotpy sim```

A python window will open with the simulation GUI. (See the [docs](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/simulation-gui.html#running-the-gui) starting at ```Using the GUI```)
## Deploying to the robot
To deploy to the robot, run 

```robotpy deploy```

This will take some time. You may be prompted to upgrade the roborio versions of packages. You most likely should do this. If you are not sure, consult the [docs](https://docs.wpilib.org) and the rest of your programming team

