// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;

/**
 * The subsystem for the rotary arM
 */
public class RotaryArm extends SubsystemBase {
  private double outputSpeed = 0.6; // the speed we will run the rotary arm at
  private final WPI_TalonFX rotaryArmMotor; // motor for the rotary arm
  private final DigitalInput m_LimitSwitch;


  /**
   *
   * Constructs a rotary arm in brake mode with 9 volts, voltage compensation
   */
  public RotaryArm() {
    rotaryArmMotor = new WPI_TalonFX(Constants.CAN_RotaryArm);
    rotaryArmMotor.configFactoryDefault();
    rotaryArmMotor.configVoltageCompSaturation(Constants.kMaxSteerVoltage);
    rotaryArmMotor.enableVoltageCompensation(true);
    m_LimitSwitch = new DigitalInput(Constants.PUNCHY_LIMIT_SWITCH);
    rotaryArmMotor.setInverted(true);
    rotaryArmMotor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * This method will be called once per scheduler run
   */
  @Override
  public void periodic() {}

  /**
   * Rotates the rotary arm
   * @param scalar value that scales the output speed from shuffleboard
   */
  public void rotateRotaryArm(double scalar){
    rotaryArmMotor.set(outputSpeed * scalar);
  }

  /**
   * This method will be called once per scheduler run during simulation
   */
  @Override
  public void simulationPeriodic() {}

  public boolean hitLimitSwitch() {
    return !m_LimitSwitch.get();
  }

  /**
   * update the output speed, usually from network tables
   * @param newSpeed the new speed to set the output speed to
   */
  protected void updateOutputSpeed(double newSpeed) {
    outputSpeed = newSpeed;
  }

  /**
   * @return the output speed for the rotary arm
   */
  protected double getOutputSpeed() {
    return outputSpeed;
  }

  /**
   * Initializes the sendable builder to put on shuffleboard
   * @param builder sendable builder
   */
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Rotary arm");
    builder.addDoubleProperty("Rotary % output", this::getOutputSpeed, this::updateOutputSpeed);
  }
}
