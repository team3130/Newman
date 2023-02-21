// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;

public class ExtensionArm extends SubsystemBase implements Sendable {
  private static double extensionArmSpeed = 1;

  private WPI_TalonSRX extensionMotor;

  // limit switch
  private final DigitalInput m_digitalInput;

  public ExtensionArm() {
    extensionMotor = new WPI_TalonSRX(Constants.CAN_ExtensionArm);
    extensionMotor.configFactoryDefault();
    extensionMotor.configVoltageCompSaturation(Constants.kMaxSteerVoltage);
    extensionMotor.enableVoltageCompensation(false); //TODO: change when we get falcon
    extensionMotor.setInverted(false);
    extensionMotor.setNeutralMode(NeutralMode.Brake);

    m_digitalInput = new DigitalInput(Constants.PUNCHY_LIMIT_SWITCH);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * spins the extension arm
   * @param scalar to scale the output speed
   */
  public void spinExtensionArm(double scalar) {
    extensionMotor.set(scalar);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public boolean hitLimitSwitch() {
    return !m_digitalInput.get();
  }

  /**
   * Stops the devices connected to this subsystem
   */
  public void stop() {
    extensionMotor.set(0);
  }

  public double getSpeed() {
    return extensionArmSpeed;
  }

  public void updateSpeed(double newSpeed) {
    extensionArmSpeed = newSpeed;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Extension % out", this::getSpeed, this::updateSpeed);
    builder.addBooleanProperty("Hit limit switch", this::hitLimitSwitch, null);
  }

}
