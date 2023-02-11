// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RotaryArm;

/** An example command that uses an example subsystem. */
public class MoveRotaryArm extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RotaryArm m_rotaryArm;
  private int dir;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveRotaryArm(RotaryArm subsystem, int direction) {
    // specify whether rotary arm should be lowered or raised by setting the direction parameter as either -1 or 1, respectively
    m_rotaryArm = subsystem;
    dir = direction;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_rotaryArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_rotaryArm.RotateRotaryArm(.25 * dir);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_rotaryArm.RotateRotaryArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
