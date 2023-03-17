// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Manipulator;

/** A command to actuate the manipulator that uses manipulator. */
public class ActuateHandGrabber extends InstantCommand {
  private final Manipulator m_manipulator;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ActuateHandGrabber(Manipulator subsystem) {
    m_manipulator = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // when button is pressed, hand is toggled
    m_manipulator.toggleGrabber();
  }
}
