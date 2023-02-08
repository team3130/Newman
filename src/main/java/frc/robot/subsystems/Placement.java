// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenixpro.hardware.TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Placement extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public WPI_TalonFX rotaryArmMotor;
  public WPI_TalonFX extensionMotor;
  public Solenoid grabber;
  public Placement() {
    rotaryArmMotor = new WPI_TalonFX(Constants.CAN_RotaryArm);
    extensionMotor = new WPI_TalonFX(Constants.CAN_ExtensionMotor);
    rotaryArmMotor.configFactoryDefault();
    extensionMotor.configFactoryDefault();
    grabber = new Solenoid(Constants.CAN_PNM, PneumaticsModuleType.CTREPCM,Constants.PNM_Grabber);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void RaiseRotaryArm(double speed){
    rotaryArmMotor.set(speed);
  }
  public void LowerRotaryArm(double speed){
    rotaryArmMotor.set(-speed);
  }
  public void ExtendExtensionArm(double speed){
    extensionMotor.set(speed);
  }
  public void RetractExtensionArm(double speed){
    extensionMotor.set(-speed);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
