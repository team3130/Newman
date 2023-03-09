// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;

/**
 * Extension arm is the punchy arm on the robot
 */
public class ExtensionArm extends SubsystemBase {
  /**
   * Speed to run the motor at by default, can be changed in shuffleboard
   */
  private static double extensionArmSpeed = 1;

  /**
   * The motor/speed controller for the mechanism
   */
  private final WPI_TalonSRX extensionMotor;

  // limit switch
  private final DigitalInput m_LimitSwitch;

  /**
   * Initializes the extension arm and configures the necessary device settings.
   * Motors are set to: Factory default, then given 9 volts of voltage compensation, and put in brake mode
   */
  public ExtensionArm() {
    extensionMotor = new WPI_TalonSRX(Constants.CAN_ExtensionArm);
    extensionMotor.configFactoryDefault();
    extensionMotor.configVoltageCompSaturation(Constants.kMaxSteerVoltage);

    extensionMotor.enableVoltageCompensation(true);
    m_LimitSwitch = new DigitalInput(Constants.PUNCHY_LIMIT_SWITCH);

    extensionMotor.enableVoltageCompensation(false); //TODO: change when we get falcon
    extensionMotor.setInverted(false);
    extensionMotor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * This method will be called once per scheduler run
   */
  @Override
  public void periodic() {}

  /**
   * spins the extension arm
   * @param scalar to scale the output speed
   */
  public void spinExtensionArm(double scalar) {
    extensionMotor.set(scalar);
  }

  /**
   * This method will be called once per scheduler run during simulation
   */
  @Override
  public void simulationPeriodic() {}

  /**
   * Whether we hit the limit switch.
   * If we hit the limit switch you are completely retracted.
   * @return whether we hit the limit switch
   */
  public boolean hitLimitSwitch() {
    return !m_LimitSwitch.get();
  }

  /**
   * Stops the devices connected to this subsystem
   */
  public void stop() {
    extensionMotor.set(0);
  }

  /**
   * returns the speed we are currently running the motor at.
   * @return the control speed of the motor
   */
  public double getSpeed() {
    return extensionArmSpeed;
  }

  /**
   * Setter for the speed
   * @param newSpeed speed to set the arm to when we run it
   */
  public void updateSpeed(double newSpeed) {
    extensionArmSpeed = newSpeed;
  }

  /**
   * Initializes the sendable object in order to update the variables.
   * @param builder sendable builder
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Extension arm");
    builder.addDoubleProperty("Extension % out", this::getSpeed, this::updateSpeed);
    builder.addBooleanProperty("Hit limit switch", this::hitLimitSwitch, null);
  }

}
