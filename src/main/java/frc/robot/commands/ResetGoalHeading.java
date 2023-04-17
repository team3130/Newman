// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ExampleSubsystem;

/** An example command that uses an example subsystem. */
public class ResetGoalHeading extends CommandBase {

  /**
   * The subsystem that this command requires
   */
  private final Chassis m_chassis;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ResetGoalHeading(Chassis subsystem) {
    m_chassis = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  /**
   * Called when the command is initially scheduled.
   */
  @Override
  public void initialize() {
    m_chassis.resetGoalHeading();
  }

  /**
   * Called every time the scheduler runs while the command is scheduled.
   */
  @Override
  public void execute() {}

  /**
   * Called once the command ends or is interrupted.
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {}

  /**
   * Returns true when the command should end.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}