// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RotaryArm;

/** An example command that uses an example subsystem. */
public class ToggleBrake extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RotaryArm m_rotary;

  /**
   * Creates a new ExampleCommand.
   *
   * @param rotary The subsystem used by this command.
   */
  public ToggleBrake(RotaryArm rotary) {
    m_rotary = rotary;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rotary);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_rotary.toggleBrake();
  }

}
