// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Manipulator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Manipulator;

/**
 * An Instant command to un-clamp the manipulator
 */
public class UnClampManipulator extends InstantCommand {
  /**
   * the singleton for manipulator. Is required by this subsystem
   */
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Manipulator m_manipulator;

  /**
   * Creates a new UnClampManipulator object
   *
   * @param subsystem The subsystem used by this command.
   */
  public UnClampManipulator(Manipulator subsystem) {
    m_manipulator = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  /**
   * Retracts the manipulator
   */
  @Override
  public void initialize() {
     m_manipulator.retract();
  }
}
