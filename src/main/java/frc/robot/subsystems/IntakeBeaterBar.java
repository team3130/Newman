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
  private WPI_TalonSRX m_beaterBar;
  private double outputSpeed = 0.5;
  public IntakeBeaterBar() {
    m_beaterBar = new WPI_TalonSRX(Constants.CAN_SpinnyBar);
    m_beaterBar.configFactoryDefault();
    m_beaterBar.configVoltageCompSaturation(Constants.kMaxSteerVoltage);
    m_beaterBar.enableVoltageCompensation(true);

    m_beaterBar.setInverted(false);
  }
  public void spin() {
    m_beaterBar.set(outputSpeed);
  }
  public void reverse(){
    m_beaterBar.set(ControlMode.PercentOutput, -outputSpeed);
  }
  public double getOutputSpeed(){
    return outputSpeed;
  }
  public void updateOutputSpeed(double newSpeed){
    outputSpeed = newSpeed;
  }
  public boolean isSpinning(){
    if(m_beaterBar.get()!=0){
      return true;
    }
    else {
      return false;
    }
  }
  public void stop(){m_beaterBar.set(ControlMode.PercentOutput, 0);}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Intake");
    builder.addDoubleProperty("Intake %", this::getOutputSpeed, this::updateOutputSpeed);
  }
}
