// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;

import swervelib.SwerveInputStream;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final         CommandXboxController operatorXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final CoralSubsystem coral = new CoralSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                  () -> Math.pow(driverXbox.getLeftY(), 3),
                                                                  () -> Math.pow(driverXbox.getLeftX(), 3))
                                                              .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                                                              .deadband(OperatorConstants.DEADBAND)
                                                              .scaleTranslation(0.8)
                                                              .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {

      UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setResolution(320, 240); // Optional: Adjust resolution
        camera.setFPS(15); //Optional: Adjust FPS

    NamedCommands.registerCommand("Feeder", elevator.Feeder());
    NamedCommands.registerCommand("L1", elevator.L1());
    NamedCommands.registerCommand("L2", elevator.L2());
    NamedCommands.registerCommand("L3", elevator.L3());
    NamedCommands.registerCommand("L4", elevator.L4());
    NamedCommands.registerCommand("Shoot", coral.onSingle());
    NamedCommands.registerCommand("Retract", coral.offSingle());

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
      driverXbox.a().onTrue(Commands.runOnce(drivebase::zeroGyro));
      driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());

      //operatorXbox.povUp().onTrue(coral.onSingle());
      //operatorXbox.povDown().onTrue(coral.offSingle());
      //operatorXbox.povLeft().onTrue(coral.forwardDouble());
      //operatorXbox.povRight().onTrue(coral.reverseDouble());


      operatorXbox.start().onTrue(elevator.L4());
      operatorXbox.y().onTrue(elevator.L3());
      operatorXbox.x().onTrue(elevator.L2());
      operatorXbox.b().onTrue(elevator.L1());
      operatorXbox.a().onTrue(elevator.Feeder());

  //      Solenoid 1 Toggle (Left Bumper Button)
      final boolean[] solenoid1State = {false}; // Initialize state to false (reverse)

      operatorXbox.leftBumper().onTrue(Commands.runOnce(() -> {
      solenoid1State[0] = !solenoid1State[0]; // Toggle the state

      if (solenoid1State[0]) {
      coral.forwardDouble1().schedule(); // If true, go forward
    } else {
      coral.reverseDouble1().schedule(); // If false, go reverse
    }
  }));

       //Solenoid 2 Toggle (Right Bumpetr Button)
      final boolean[] solenoid2State = {false};
    driverXbox.rightBumper().onTrue(Commands.runOnce(() -> {
    solenoid2State[0] = !solenoid2State[0];
    if (solenoid2State[0]) {
        coral.forwardDouble2().schedule();
    } else {
        coral.reverseDouble2().schedule();
    }
  }));


  //Solenoid 3 Toggle (Right Bumpetr Button)
  final boolean[] solenoid3State = {false};
  operatorXbox.back().onTrue(Commands.runOnce(() -> {
  solenoid3State[0] = !solenoid3State[0];
  if (solenoid3State[0]) {
      coral.forwardDouble3().schedule();
  } else {
      coral.reverseDouble3().schedule();
  }
}));
      //only have double soleniods - ended up adding code above to create a toggle
      //for one button peration to allow other buttons to be used elswhere 
      //code below was just to test the double soleniods
      // they have been put back to control elevator
      //operatorXbox.y().onTrue(coral.forwardDouble1()); 
      //operatorXbox.x().onTrue(coral.reverseDouble1());
      //operatorXbox.b().onTrue(coral.forwardDouble2());
      //operatorXbox.a().onTrue(coral.reverseDouble2()); 


      // Operator Left Y-Axis Control for Elevator
      double elevatorDeadZone = 0.1; // Adjust this value as needed

      new Trigger(() -> Math.abs(operatorXbox.getLeftY()) > elevatorDeadZone)
              .whileTrue(elevator.controlElevatorManual(() -> {
                  double yAxis = operatorXbox.getLeftY();
                  if (Math.abs(yAxis) < elevatorDeadZone) {
                      return 0.0;
                  } else {
                      return yAxis * -1;
                  }
              }))
              .whileFalse(elevator.stopElevatorManual());


    // Variable 775 Pro Motor Speed Control with Operator Xbox Y
    new Trigger(() -> Math.abs(operatorXbox.getRightY()) > 0.1)
        .whileTrue(elevator.control775ProMotor(operatorXbox::getRightY))
        .whileFalse(elevator.stop775ProMotor());

    // Variable 775 Pro Motor Speed Control with Operator Xbox Y and Dead Zone
    double deadZone = 0.1; // Adjust this value as needed

    new Trigger(() -> Math.abs(operatorXbox.getRightY()) > deadZone)
        .whileTrue(elevator.control775ProMotor(() -> {
            double yAxis = operatorXbox.getRightY();
            if (Math.abs(yAxis) < deadZone) {
                return 0.0; // Inside dead zone, return 0
            } else {
                return yAxis; // Outside dead zone, return the axis value
            }
        }))
        .whileFalse(elevator.stop775ProMotor());


    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
