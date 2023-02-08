// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

/** An example command that uses an example subsystem. */
public class GoToAngle extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Chassis m_subsystem;
  private SwerveModuleState desiredState;
  private SwerveModuleState[] states;

  private final double point;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GoToAngle(Chassis subsystem, double point) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.point = point;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.turnToAngle(point);
    desiredState = new SwerveModuleState();
    states = new SwerveModuleState[]{desiredState, desiredState, desiredState, desiredState};
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  m_subsystem.setModuleStates(states);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
