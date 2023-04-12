// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Manipulator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Manipulator;

/** An Instant command that clamps the manipulator (Also called grabber) */
public class ClampManipulator extends InstantCommand {
  /**
   * The singleton for manipulator. gets required by this subsystem
   */
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Manipulator m_manipulator;

  /**
   * Creates a new ClampManipulator object
   *
   * @param manipulator the manipulator singleton for actuate the manipulator's pneumatics. Gets required by the command
   */
  public ClampManipulator(Manipulator manipulator) {
    m_manipulator = manipulator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(manipulator);
  }

  /**
   * Clamps the manipulator aka extends the manipulator
   */
  @Override
  public void initialize() {
     m_manipulator.retract();
  }
}
