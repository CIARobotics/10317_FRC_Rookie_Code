package frc.robot.subsystems.Solenoid;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SolenoidSubsystem extends TimedRobot {
    private final Joystick m_stick = new Joystick(1);
    private final Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
    private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);

    private final int revPHCanID = 2; // CAN ID 2
    private final Compressor m_compressor = new Compressor(revPHCanID, PneumaticsModuleType.REVPH);

    static final int kSolenoidButton = 1;
    static final int kDoubleSolenoidForwardButton = 2;
    static final int kDoubleSolenoidReverseButton = 3;
    static final int kCompressorButton = 4;

    public SolenoidSubsystem() {
        ShuffleboardTab tab = Shuffleboard.getTab("Pneumatics");
        tab.add("Single Solenoid", m_solenoid);
        tab.add("Double Solenoid", m_doubleSolenoid);
        tab.add("Compressor", m_compressor);
        tab.addDouble("PH Pressure [PSI]", m_compressor::getPressure);
        tab.addDouble("Compressor Current", m_compressor::getCurrent);
        tab.addBoolean("Compressor Active", m_compressor::isEnabled);
        tab.addBoolean("Pressure Switch", m_compressor::getPressureSwitchValue);
    }

    @SuppressWarnings("PMD.UnconditionalIfStatement")
    @Override
    public void teleopPeriodic() {
        m_solenoid.set(m_stick.getRawButton(kSolenoidButton));

        if (m_stick.getRawButtonPressed(kDoubleSolenoidForwardButton)) {
            m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
        } else if (m_stick.getRawButtonPressed(kDoubleSolenoidReverseButton)) {
            m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
        }

        if (m_stick.getRawButtonPressed(kCompressorButton)) {
            boolean isCompressorEnabled = m_compressor.isEnabled();
            if (isCompressorEnabled) {
                m_compressor.disable();
            } else {
                if (false) {
                    m_compressor.enableDigital();
                }
                if (true) {
                    m_compressor.enableAnalog(70, 120);
                }
                if (false) {
                    m_compressor.enableHybrid(70, 120);
                }
            }
        }
    }
}