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

    extensionMotor.configVoltageCompSaturation(Constants.kMaxExtensionArmVoltage);
    extensionMotor.enableVoltageCompensation(true);

    extensionMotor.setInverted(false);
    extensionMotor.setNeutralMode(NeutralMode.Brake);

    m_LimitSwitch = new DigitalInput(Constants.PUNCHY_LIMIT_SWITCH);


  }

  /**
   * This method will be called once per scheduler run
   */
  @Override
  public void periodic() {}



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
