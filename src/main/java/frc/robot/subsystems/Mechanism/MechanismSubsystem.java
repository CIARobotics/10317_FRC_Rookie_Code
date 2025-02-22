package frc.robot.subsystems.Mechanism;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class MechanismSubsystem extends SubsystemBase {
    private SparkMax m_leftMotor;
    private SparkMax m_rightMotor;
    private RelativeEncoder m_leftEncoder;
    private RelativeEncoder m_rightEncoder;

    public MechanismSubsystem() {
        m_leftMotor = new SparkMax(20, MotorType.kBrushless); // Replace with your CAN IDs
        m_rightMotor = new SparkMax(21, MotorType.kBrushless); // Replace with your CAN IDs

        m_leftEncoder = m_leftMotor.getEncoder();
        m_rightEncoder = m_rightMotor.getEncoder();
        
        //Set the position of the encoder to 0, if you want it to start at 0
        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);
    }

    public void setLeftMotorSpeed(double speed) {
        m_leftMotor.set(speed);
    }

    public void setRightMotorSpeed(double speed) {
        m_rightMotor.set(speed);
    }

    public void handleButtonInputs(boolean buttonA, boolean buttonB) {
        if (buttonA) {
            // Action for button A (e.g., toggle something)
            System.out.println("Button A Pressed");
        }
        if (buttonB) {
            // Action for button B
            System.out.println("Button B Pressed");
        }
    }

    public double getLeftEncoder() {
        return m_leftEncoder.getPosition();
    }

    public double getRightEncoder() {
        return m_rightEncoder.getPosition();
    }

    @Override
    public void periodic() {
        // This method is called periodically (e.g., every 20ms)
        // You can add code here for continuous tasks or sensor updates.
    }
}