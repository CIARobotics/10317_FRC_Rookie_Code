// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {
  private final Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
  private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);

  private final int revPHCanID = 2; // CAN ID 2
  private final Compressor m_compressor = new Compressor(revPHCanID, PneumaticsModuleType.REVPH);
  /** Creates a new Coral. */
  public CoralSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Pneumatics");
    tab.add("Single Solenoid", m_solenoid);
    tab.add("Double Solenoid", m_doubleSolenoid);
    tab.add("Compressor", m_compressor);
    tab.addDouble("PH Pressure [PSI]", m_compressor::getPressure);
    tab.addDouble("Compressor Current", m_compressor::getCurrent);
    tab.addBoolean("Compressor Active", m_compressor::isEnabled);
    tab.addBoolean("Pressure Switch", m_compressor::getPressureSwitchValue);
  }

  // I don't know what you want to call these please rename
  public Command onSingle() {
    return Commands.runOnce(() -> m_solenoid.set(true));
  }

  public Command offSingle() {
    return Commands.runOnce(() -> m_solenoid.set(false));
  }

  public Command forwardDouble() {
    return Commands.runOnce(() -> m_doubleSolenoid.set(Value.kForward));
  }

  public Command reverseDouble() {
    return Commands.runOnce(() -> m_doubleSolenoid.set(Value.kReverse));
  }
}
