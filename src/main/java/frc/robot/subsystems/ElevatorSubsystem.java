// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorSetpoints;

public class ElevatorSubsystem extends SubsystemBase {

     // Limit Switch
     private DigitalInput lowerLimitSwitch = new DigitalInput(1);
     private DigitalInput upperLimitSwitch = new DigitalInput(2); 


  /** Subsystem-wide setpoints */
  public enum Setpoint {
    kFeederStation,
    kLevel1,
    kLevel2,
    kLevel3,
    kLevel4;
  }

  // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.
  private SparkMax elevatorMotor =
      new SparkMax(20, MotorType.kBrushless);
  //private SparkMax elevatorMotor2 = 
      //new SparkMax(deviceId:21, MotorType.kBrushless);
      //might not need - set elevatorMotor2 to follower in REV Hardware Client
  private SparkClosedLoopController elevatorClosedLoopController =
      elevatorMotor.getClosedLoopController();
  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

  private double elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
  private SparkMaxConfig elevatorConfig = new SparkMaxConfig();

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(50).voltageCompensation(12);

    /*
      * Configure the reverse limit switch for the elevator. By enabling the limit switch, this
      * will prevent any actuation of the elevator in the reverse direction if the limit switch is
      * pressed.
      */
    elevatorConfig
        .limitSwitch
        .reverseLimitSwitchEnabled(true)
        .reverseLimitSwitchType(Type.kNormallyOpen);

    /*
      * Configure the closed loop controller. We want to make sure we set the
      * feedback sensor as the primary encoder.
      */
    elevatorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control
        .p(0.08)
.outputRange(-1, 1)
.maxMotion
// Set MAXMotion parameters for position control
.maxVelocity(4200)
.maxAcceleration(6000)
.allowedClosedLoopError(0.5);

    elevatorMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Upper Limit Switch", upperLimitSwitch.get());
    SmartDashboard.putBoolean("Lower Limit Switch", lowerLimitSwitch.get());
    SmartDashboard.putNumber("Coral/Elevator/Target Position", elevatorCurrentTarget);
    SmartDashboard.putNumber("Coral/Elevator/Actual Position", elevatorEncoder.getPosition());
  }

  public Command controlElevatorManual(java.util.function.DoubleSupplier speedSupplier) {
    return Commands.run(() -> {
        double speed = speedSupplier.getAsDouble();
        if (!lowerLimitSwitch.get() && speed < 0) {
            elevatorMotor.set(0);
            elevatorEncoder.setPosition(0);
        } else if (!upperLimitSwitch.get() && speed > 0) { // Check top limit switch
            elevatorMotor.set(0); // Stop if top limit switch is triggered and moving up
            elevatorEncoder.setPosition(162);
        } else {
            elevatorMotor.set(speed);
        }
    });
  }

  public Command L4() {
    return Commands.runOnce(() -> moveToSetpoint(ElevatorSetpoints.kLevel4));
  }

  public Command L3() {
    return Commands.runOnce(() -> moveToSetpoint(ElevatorSetpoints.kLevel3));
  }

  public Command L2() {
    return Commands.runOnce(() -> moveToSetpoint(ElevatorSetpoints.kLevel2));
  }

  public Command L1() {
    return Commands.runOnce(() -> moveToSetpoint(ElevatorSetpoints.kLevel1));
  }

 public Command Feeder() {
   return Commands.runOnce(() -> moveToSetpoint(ElevatorSetpoints.kFeederStation));
  }

  private void moveToSetpoint(int target) {
    elevatorClosedLoopController.setReference(
        target, ControlType.kMAXMotionPositionControl);
  }

  
// 775 Pro Motor and PWM Spark
private PWMSparkMax motor775Pro = new PWMSparkMax(1);


  // 775 Pro Motor Control
  private void set775ProMotor(double speed) {
      motor775Pro.set(speed);
  }

  public Command control775ProMotor(java.util.function.DoubleSupplier speedSupplier) {
      return Commands.run(() -> set775ProMotor(speedSupplier.getAsDouble()));
  }

  public Command stop775ProMotor() {
      return Commands.runOnce(() -> set775ProMotor(0.0));
  }


//Add this method
public Command stopElevatorManual() {
    return Commands.runOnce(() -> elevatorMotor.set(0.0));
}

  
}
