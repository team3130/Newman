// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Manipulator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Manipulator;

/**
 * An instant command the toggles the manipulator
 */
public class ToggleManipulator extends InstantCommand {
  /**
   * the singleton for manipulator. Is required by this subsystem
   */
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Manipulator m_manipulator;

  /**
   * Creates a new ToggleManipulator command.
   *
   * @param manipulator The subsystem used by this command.
   */
  public ToggleManipulator(Manipulator manipulator) {
    m_manipulator = manipulator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(manipulator);
  }

  /**
   * Toggles the manipulator solenoid which toggles the pistons states.
   */
  @Override
  public void initialize() {
     m_manipulator.toggleManipulator();
  }
}
