// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis.presets;

import frc.robot.subsystems.Chassis;
import frc.robot.controls.AutomaticallyGoToALocation;
import frc.robot.supportingClasses.Auton.AutonManager;

/**
 * A command to go to the human player station automatically.
 * Makes an object that sets the parent's ({@link AutomaticallyGoToALocation}) runnable.
 */
public class GoToHumanPlayerStation extends AutomaticallyGoToALocation {

  /**
   * Makes a new GoToHumanPlayerStation object. calls the super's constructor to configure the runnable
   *
   * @param m_chassis the chassis subsystem which gets required and used for the current position of the bot
   * @param manager the auton manager singleton which is used to generate the path as a runnable
   */
  public GoToHumanPlayerStation(Chassis m_chassis, AutonManager manager) {
    super(m_chassis, () -> manager.makeCmdToGoToHumanPlayerStation(m_chassis.getPose2d()));
  }

}
