// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;

/**
 * The hopper subsystem that can be put on shuffleboard.
 * contains two motors that run in opposite directions.
 * Because build is un-organized the right wheel is geared 1/4 of the left wheel
 */
public class Hopper extends SubsystemBase {
    protected final WPI_TalonSRX m_leftWheel; // the left wheel for hopper
    protected final WPI_TalonSRX m_rightWheel; // the right wheel for hopper
    private double m_leftHopperSpeed = 0.45; // the left output speed
    private double m_rightHopperSpeed = m_leftHopperSpeed; //TODO double check gearing is the same
    private double m_shootingOutputSpeed = -0.5;

    /**
     * Constructs a hopper with 9 volts of voltage compensation on the motors.
     */
    public Hopper() {
        m_leftWheel = new WPI_TalonSRX(Constants.CAN_hopperleft);
        m_rightWheel = new WPI_TalonSRX(Constants.CAN_hopperright);
        m_leftWheel.configFactoryDefault();
        m_rightWheel.configFactoryDefault();
        m_rightWheel.configVoltageCompSaturation(Constants.kMaxSteerVoltage);
        m_rightWheel.enableVoltageCompensation(true);
        m_leftWheel.configVoltageCompSaturation(Constants.kMaxSteerVoltage);
        m_leftWheel.enableVoltageCompensation(true);

        m_leftWheel.setInverted(true);
        m_rightWheel.setInverted(false);
    }

    /**
     * spin the motor at the set speed which can be updated from shuffleboard or with the {@link #updateOutputSpeed(double speed)} method
     */
    public void spinHopper() {
        m_leftWheel.set(ControlMode.PercentOutput, m_leftHopperSpeed);
        m_rightWheel.set(ControlMode.PercentOutput, m_rightHopperSpeed);
    }

    /** spin fast enough to shoot **/
    public void spinToShoot(){
        m_leftWheel.set(ControlMode.PercentOutput, m_shootingOutputSpeed);
        m_rightWheel.set(ControlMode.PercentOutput, m_shootingOutputSpeed);
    }
    /** alternating directions to unjam pieces that have become lodged **/
    public void alternateHopper(){
        m_leftWheel.set(ControlMode.PercentOutput,-m_leftHopperSpeed);
        m_rightWheel.set(ControlMode.PercentOutput, m_rightHopperSpeed);
    }

    /**
     * Reverse the direction of the hopper motors to feed out
     */
    public void reverseHopper() {
        m_leftWheel.set(ControlMode.PercentOutput, -m_leftHopperSpeed);
        m_rightWheel.set(ControlMode.PercentOutput, -m_rightHopperSpeed);
    }

    /**
     * stop the motors, usually called when commands end
     */
    public void stopHopper() {
        m_leftWheel.set(ControlMode.PercentOutput, 0);
        m_rightWheel.set(ControlMode.PercentOutput, 0);
    }

    /**
     * setter for the output speed.
     * The right side is geared wrong, so it needs to run at 4x the left side
     * @param newSpeed to run the motors at
     */
    public void updateOutputSpeed(double newSpeed) {
        m_leftHopperSpeed = newSpeed;
        m_rightHopperSpeed = newSpeed * 4;
    }

    /**
     * @return the left side set speed
     */
    public double getSpeed() {
        return m_leftHopperSpeed;
    }

    /**
     * This method will be called once per scheduler run
     */
    @Override
    public void periodic() {}

    /**
     * This method will be called once per scheduler run during simulation
     */
    @Override
    public void simulationPeriodic() {}

    /**
     * Initializes the sendable for shuffleboard
     * @param builder sendable builder
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Hopper");
        builder.addDoubleProperty("Hopper % out", this::getSpeed, this::updateOutputSpeed);
    }
}