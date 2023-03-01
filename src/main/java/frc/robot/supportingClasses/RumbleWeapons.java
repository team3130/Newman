// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.supportingClasses;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.RobotContainer.m_weaponsGamepad;
import static frc.robot.Newman_Constants.Constants.kExtensionArmLength;


/** An example command that uses an example subsystem. */
public class RumbleWeapons extends CommandBase {


  public RumbleWeapons(RumbleWeapons subsystem) {
    if (RobotState.isEnabled()) {
      m_weaponsGamepad.setRumble(GenericHID.RumbleType.kBothRumble, (1/kExtensionArmLength));//replace 1 with variable for arm length currently
    } else {
      m_weaponsGamepad.setRumble(GenericHID.RumbleType.kBothRumble, 0);
    }
  }
}