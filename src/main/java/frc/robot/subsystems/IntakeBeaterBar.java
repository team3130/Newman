// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;

public class IntakeBeaterBar extends SubsystemBase {
  private final WPI_TalonSRX m_beaterBar;

  private double outputSpeed = 0.5;

  public IntakeBeaterBar() {
    m_beaterBar = new WPI_TalonSRX(Constants.CAN_SpinnyBar);
    m_beaterBar.configFactoryDefault();
    m_beaterBar.configVoltageCompSaturation(Constants.kMaxSteerVoltage);
    m_beaterBar.enableVoltageCompensation(true);

    m_beaterBar.setInverted(false);
  }

  /**
   * Spin the intake at {@link #outputSpeed}
   */
  public void spin() {
    m_beaterBar.set(ControlMode.PercentOutput, outputSpeed);
  }

  /**
   * Reverse the intake at the same speed
   */
  public void reverse(){
    m_beaterBar.set(ControlMode.PercentOutput, -outputSpeed);
  }

  /**
   * @return whatever the current set speed is for intake
   */
  public double getOutputSpeed(){
    return outputSpeed;
  }

  /*
   * @param newSpeed the new speed to spin the intake at
   */
  public void updateOutputSpeed(double newSpeed) {
    outputSpeed = newSpeed;
  }

  /**
   * @return whether the motor has an output or not
   */
  public boolean isSpinning() {
    return m_beaterBar.get() != 0;
  }

  /**
   * Stops the beater bar
   */
  public void stop() {
    m_beaterBar.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Intake");
    builder.addDoubleProperty("Intake %", this::getOutputSpeed, this::updateOutputSpeed);
  }
}
