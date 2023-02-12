// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;

public class Manipulator extends SubsystemBase {
  public Solenoid grabber;
  /** Creates a new Subsystem. */
  public Manipulator() {
    grabber = new Solenoid(Constants.CAN_PNM, PneumaticsModuleType.CTREPCM, Constants.PNM_Grabber);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void toggleManipulator(){
    grabber.toggle();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
