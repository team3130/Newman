// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakePivot;

/** A command to get intake to go to the next position. uses Intake as subsystem. */
public class GoToNextIntakePos extends InstantCommand {
  private final IntakePivot m_pivot;

  /**
   * Creates a new GoToNextIntakePos command
   *
   * @param pivot the intake pivot subsystem used by this command
   */
  public GoToNextIntakePos(IntakePivot pivot) {
    m_pivot = pivot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pivot.goToNext();
  }
}
