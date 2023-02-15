// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;

public class ExtensionArm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public WPI_TalonSRX extensionMotor;
  public DigitalInput m_LimitSwitch;
  public ExtensionArm() {
    extensionMotor = new WPI_TalonSRX(Constants.CAN_ExtensionArm);
    extensionMotor.configFactoryDefault();
    extensionMotor.configVoltageCompSaturation(Constants.kMaxSteerVoltage);
    extensionMotor.enableVoltageCompensation(true);
    m_LimitSwitch = new DigitalInput(Constants.PUNCHY_LIMIT_SWITCH);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void ExtendExtensionArm(double speed) {
    extensionMotor.set(speed);
  }
  public void StopArm() {
    extensionMotor.set(0);
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
