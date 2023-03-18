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

  public IntakeBeaterBar() {
    m_beaterBar = new WPI_TalonSRX(Constants.CAN_SpinnyBar);
    m_beaterBar.configFactoryDefault();
    m_beaterBar.configVoltageCompSaturation(Constants.kMaxVoltageIntakeBeaterBar);
    m_beaterBar.enableVoltageCompensation(true);

    m_beaterBar.setInverted(true);
  }

  /**
   * Spin the intake at 100 %
   */
  public void spin() {
    m_beaterBar.set(ControlMode.PercentOutput, 1);
  }

  /**
   * Reverse the intake at the same speed
   */
  public void reverse(){
    m_beaterBar.set(ControlMode.PercentOutput, -1);
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
}
