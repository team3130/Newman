# ERRORs code for 2023
This year we are using a Swerve Drive Robot so we implemented WPILIBs Swerve Drive Kinematics classes

## Newman Constants are in a submodule
This code repository depends on a submodule with shared constants.
To pull the submodule into the project, assuming the project is newly cloned from GiHub, do the following:

1. `cd src/main/java/frc/robot/Newman_Constants/`
2. `git submodule init`
3. `git submodule update`

Then it should build
