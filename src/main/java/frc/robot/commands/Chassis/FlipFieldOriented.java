// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Chassis;

/**
 * A command that flips between robot and field oriented.
 * Requires chassis
 */
public class FlipFieldOriented extends InstantCommand {

  /**
   * The singleton for chassis. Is the only required subsystem for this command
   */
  private final Chassis m_chassis;

  /**
   * Creates a new command to flip between field and robot oriented control
   *
   * @param chassis The chassis subsystem
   */
  public FlipFieldOriented(Chassis chassis) {
    m_chassis = chassis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
  }

  /**
   * Called once when the scheduler runs for the first time
   */
  @Override
  public void initialize() {
    m_chassis.flipFieldRelative();
  }
}
