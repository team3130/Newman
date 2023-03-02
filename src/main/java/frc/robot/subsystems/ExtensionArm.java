// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;

/**
 * The subsystem for the extension arm
 */
public class ExtensionArm extends SubsystemBase {
  /**
   * Weapons gamepad
   */
  private static Joystick gamepad;
  /**
   * mechanism 2d to show the extension arm length
   */
  protected MechanismLigament2d ligament;
  /**
   * Speed to run the motor at by default, can be changed in shuffleboard
   */
  private static ShuffleboardTab tab = Shuffleboard.getTab("Placement");
  private double extensionArmSpeed = 1;

  /**
   * The motor/speed controller for the mechanism
   */
  private final WPI_TalonSRX extensionMotor;

  // limit switch
  private final DigitalInput m_LimitSwitch;

  /**
   * Initializes the extension arm and configures the necessary device settings.
   * Motors are set to: Factory default, then given 9 volts of voltage compensation, and put in brake mode
   *
   * @param ligament the ligament object that is on smart-dashboard
   */
  public ExtensionArm(MechanismLigament2d ligament, Joystick gamepad) {
    extensionMotor = new WPI_TalonSRX(Constants.CAN_ExtensionArm);
    extensionMotor.configFactoryDefault();
    extensionMotor.configVoltageCompSaturation(Constants.kMaxSteerVoltage);
    extensionMotor.enableVoltageCompensation(true);
    m_LimitSwitch = new DigitalInput(Constants.PUNCHY_LIMIT_SWITCH);

    extensionMotor.enableVoltageCompensation(false); //TODO: change when we get falcon
    extensionMotor.setInverted(false);
    extensionMotor.setNeutralMode(NeutralMode.Brake);
    
    this.gamepad = gamepad;
    this.ligament = ligament;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    gamepad.setRumble(GenericHID.RumbleType.kBothRumble,getRumbleExtensionArmValue());
    ligament.setLength(getLengthExtensionArm());
  }

  /**
   * extend the extension arm
   * @param speed the speed to run the extension arm at
   */
  public void extendExtensionArm(double speed) {
    extensionMotor.set(speed * extensionArmSpeed);
  }

  /**
   * Stops the extension arm
   */
  public void stopArm() {
    extensionMotor.set(0);
  }

  /**
   * @return whether the limit switch got broke or not
   */
  public boolean brokeLimit(){
    return !m_LimitSwitch.get();
  }

  /**
   * @return the length of the extension arm
   */
  public double getLengthExtensionArm(){
    return Constants.kTicksToMetersExtension * extensionMotor.getSelectedSensorPosition() + Constants.kExtensionArmLengthRetracted;
  }

  /**
   * @return the value to rumble to extension arm
   */
  public double getRumbleExtensionArmValue(){
    return Math.pow(((Constants.kExtensionArmLengthExtended - Constants.kExtensionArmLengthRetracted) /
            (getLengthExtensionArm() - Constants.kExtensionArmLengthRetracted)),2);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
