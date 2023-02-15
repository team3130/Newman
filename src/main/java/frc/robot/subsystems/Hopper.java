// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Hopper extends SubsystemBase {
    protected final WPI_TalonSRX leftwheel;
    protected final WPI_TalonSRX rightwheel;

    private static ShuffleboardTab TAB = Shuffleboard.getTab("Hopper");
    private double m_outputspeed = 0.45;
    private final GenericEntry n_outputentry = TAB.add("HopperOutput", m_outputspeed).getEntry();

    public Hopper() {
        leftwheel = new WPI_TalonSRX(Constants.CAN_hopperleft);
        rightwheel = new WPI_TalonSRX(Constants.CAN_hopperright);
        leftwheel.configFactoryDefault();
        rightwheel.configFactoryDefault();
        rightwheel.configVoltageCompSaturation(Constants.kMaxSteerVoltage);
        rightwheel.enableVoltageCompensation(true);
        leftwheel.configVoltageCompSaturation(Constants.kMaxSteerVoltage);
        leftwheel.enableVoltageCompensation(true);
    }

    public void spinMotor() {
        leftwheel.set(ControlMode.PercentOutput, m_outputspeed);
        rightwheel.set(ControlMode.PercentOutput, m_outputspeed);
    }

    public void hopperStop() {
        leftwheel.set(ControlMode.PercentOutput, 0);
        rightwheel.set(ControlMode.PercentOutput, 0);
    }

    public void updateOutputSpeed() {
        m_outputspeed = n_outputentry.getDouble(m_outputspeed);
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