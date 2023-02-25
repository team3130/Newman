// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;
import frc.robot.supportingClasses.ShuffleboardUpdated;

public class ExtensionArm extends SubsystemBase implements ShuffleboardUpdated {
  private static double extensionArmSpeed = 1;
  private ShuffleboardTab tab = Shuffleboard.getTab("Test");
  private GenericEntry n_outputSpeed = tab.add("Extension % out", extensionArmSpeed).getEntry();
  private GenericEntry n_limitSwitch = tab.add("Limit switch", false).getEntry();
  public WPI_TalonSRX extensionMotor;
  public DigitalInput m_LimitSwitch;

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

  public void outputShuffleboard() {
    n_limitSwitch.setBoolean(brokeLimit());
  }

  @Override
  public void periodic() {
    outputShuffleboard();
    // This method will be called once per scheduler run
  }

  /**
   * spins the extension arm
   * @param scalar to scale the output speed
   */
  public void spinExtensionArm(double scalar) {
    extensionMotor.set(scalar);
  }
  public void StopArm() {
    extensionMotor.set(0);
  }
  public boolean brokeLimit(){
    return !m_LimitSwitch.get();
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void stop() {
    extensionMotor.set(0);
  }

  @Override
  public void updateValueFromShuffleboard() {
    extensionArmSpeed = n_outputSpeed.getDouble(extensionArmSpeed);
  }
}
