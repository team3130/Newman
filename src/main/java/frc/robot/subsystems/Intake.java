// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;

public class Intake extends SubsystemBase {
  private WPI_TalonSRX m_beaterBar;
  private WPI_TalonFX m_pivot;
  private DigitalInput m_lowPosition;
  private DigitalInput m_middlePosition;
  private DigitalInput m_highPosition;

  public Intake() {
    m_beaterBar = new WPI_TalonSRX(Constants.CAN_SpinnyBar);
    m_pivot = new WPI_TalonFX(Constants.CAN_pivot);
    m_lowPosition = new DigitalInput(Constants.DIO_LowPosition);
    m_middlePosition = new DigitalInput(Constants.DIO_MidPosition);
    m_highPosition = new DigitalInput(Constants.DIO_HighPosition);
  }
  public void SpinBeaterBar () {
    m_beaterBar.set(0.5);
    m_beaterBar.set(0);
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
