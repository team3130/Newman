// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;

public class IntakePivot extends SubsystemBase {
  private WPI_TalonFX pivotMotor;
  public DigitalInput m_lowPosition;
  public DigitalInput m_middlePosition;
  public DigitalInput m_highPosition;
  public DigitalInput lastLimitPosition;

  public IntakePivot() {
    pivotMotor = new WPI_TalonFX(Constants.CAN_pivot);
    m_lowPosition = new DigitalInput(Constants.DIO_LowPosition);
    m_middlePosition = new DigitalInput(Constants.DIO_MidPosition);
    m_highPosition = new DigitalInput(Constants.DIO_HighPosition);
  }
  public boolean hitLimitSwitch(DigitalInput limitSwitch){return !(limitSwitch.get());}
  public void stop(){pivotMotor.set(0);}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(hitLimitSwitch(m_lowPosition)){lastLimitPosition = m_lowPosition;}
    if(hitLimitSwitch(m_middlePosition)){lastLimitPosition = m_middlePosition;}
    if(hitLimitSwitch(m_highPosition)){lastLimitPosition = m_highPosition;}
  }
  public double getAngle(){return (Constants.kTicksToRadiansIntakePivotMotor * pivotMotor.getSelectedSensorPosition());}
  public void movePivotMotor(int dir){
    pivotMotor.set(Constants.kIntakePivotMotorSpeed*dir);
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
