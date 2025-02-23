# 10317_FRC_Rookie_Code
This code is only a test code and made public to allow us to ask other FRC teams and mentors for help and guidance. Please note that most of the code does not work.

Public - to allow help from any FRC teams and mentors.

The original structure and code is from https://docs.yagsl.com/
source code: https://github.com/BroncBotz3481/YAGSL-Example/tree/main/src/main
Steps from their website were followed to tune our swerve drive for our specific build and design.

Code we have changed (as suggested from their website):
src/main/java/frc/robot/subsystems/swervedrive/SwerveSubsystem.java
src/main/deploy/swerve/neo/modules

The swerve drive portion of the code is working. More tuning and testing is needed to dial in for our robot.

We have been trying to integrate other templates into this code to allow for a second controller to operate motors via a button press or toggle. We are also trying to add the pneumatic system as well for buttons to control both single and double solenoid, as well as initiate the compressor. We created two "subsytems" as follows:

src/main/java/frc/robot/subsystems/Mechanism/MechanismSubsystem.java
src/main/java/frc/robot/subsystems/Solenoid/SolenoidSubsystem.java

The code for the above java files where templates from the FRC WpiLib resources: https://docs.wpilib.org/en/stable/docs/software/examples-tutorials/wpilib-examples.html

As a rookie team, we really do not know what we are doing. We do not have the ability to code from scratch, but are trying to add code and test, modify and test, etc. to see what the code is doing. (reverse engineering our code and trying to merge template code)

The original template for the yagsl swerve drive is so advanced and large, we really do not know how to add the simpler mechanisms, solendoid, and future autonomous code. Where the code should go and is referenced is beyond our ability at this time.

We greatly appreciate any and all help for those that like to code and provide guidance. We truly thank you in advance for your help.
