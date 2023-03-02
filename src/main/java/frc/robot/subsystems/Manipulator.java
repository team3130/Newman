// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;

/**
 * subsystem for the manipulator
 */
public class Manipulator extends SubsystemBase {
  public Solenoid grabber; // solenoid for firing the pistons

  /**
   * Creates a new Manipulator
   */
  public Manipulator() {
    grabber = new Solenoid(Constants.CAN_PNM, PneumaticsModuleType.CTREPCM , Constants.PNM_Grabber);
  }

  /**
   * toggles the manipulator
   */
  public void toggleManipulator(){
    grabber.toggle();
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