// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;

public class RotaryArm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private double outputSpeed = 0.6;

  private WPI_TalonFX rotaryArmMotor;

  public RotaryArm() {
    rotaryArmMotor = new WPI_TalonFX(Constants.CAN_RotaryArm);
    rotaryArmMotor.configFactoryDefault();
    rotaryArmMotor.configVoltageCompSaturation(Constants.kMaxSteerVoltage);
    rotaryArmMotor.enableVoltageCompensation(true);
    rotaryArmMotor.setInverted(true);
    rotaryArmMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Rotates the rotary arm
   * @param scalar value that scales the output speed from shuffleboard
   */
  public void rotateRotaryArm(double scalar){
    rotaryArmMotor.set(outputSpeed * scalar);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  protected void updateOutputSpeed(double newSpeed) {
    outputSpeed = newSpeed;
  }

  protected double getOutputSpeed() {
    return outputSpeed;
  }

  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Rotary % output", this::getOutputSpeed, this::updateOutputSpeed);
  }
}
