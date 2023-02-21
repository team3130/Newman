// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Hopper extends SubsystemBase implements Sendable {
    protected final WPI_TalonSRX leftwheel;
    protected final WPI_TalonSRX rightwheel;

    private static ShuffleboardTab TAB = Shuffleboard.getTab("Hopper");
    private double m_leftoutputSpeed = 0.45;
    private double m_rightOutputSpeed = m_leftoutputSpeed * 4;
    private final GenericEntry n_leftoutputentry = TAB.add("HopperLeftOutput", m_leftoutputSpeed).getEntry();
    private final GenericEntry n_rightoutputentry = TAB.add("HopperRightOutput", m_rightOutputSpeed).getEntry();

    public Hopper() {
        leftwheel = new WPI_TalonSRX(Constants.CAN_hopperleft);
        rightwheel = new WPI_TalonSRX(Constants.CAN_hopperright);
        leftwheel.configFactoryDefault();
        rightwheel.configFactoryDefault();
        rightwheel.configVoltageCompSaturation(Constants.kMaxSteerVoltage);
        rightwheel.enableVoltageCompensation(true);
        leftwheel.configVoltageCompSaturation(Constants.kMaxSteerVoltage);
        leftwheel.enableVoltageCompensation(true);

        leftwheel.setInverted(true);
        rightwheel.setInverted(false);
    }

    public void spinMotor() {
        leftwheel.set(ControlMode.PercentOutput, m_leftoutputSpeed);
        rightwheel.set(ControlMode.PercentOutput, m_rightOutputSpeed);
    }

    public void hopperStop() {
        leftwheel.set(ControlMode.PercentOutput, 0);
        rightwheel.set(ControlMode.PercentOutput, 0);
    }

    public void updateOutputSpeed() {
        m_leftoutputSpeed = n_leftoutputentry.getDouble(m_leftoutputSpeed);
        m_rightOutputSpeed = n_rightoutputentry.getDouble(m_rightOutputSpeed) * 4;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}