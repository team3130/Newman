// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;

public class IntakeBeaterBar extends SubsystemBase {
  private WPI_TalonSRX m_beaterBar;
  public IntakeBeaterBar() {
    m_beaterBar = new WPI_TalonSRX(Constants.CAN_SpinnyBar);
  }
  public void Spin() {
    m_beaterBar.set(0.5);
  }
  public boolean isSpinning(){
    if(m_beaterBar.get()!=0){
      return true;
    }
    else {
      return false;
    }
  }
  public void Stop(){m_beaterBar.set(0);}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
