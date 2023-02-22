// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

/** A command that uses chassis to go to an angle. */
public class GoToAngle extends CommandBase {
  private final Chassis m_chassis; // the chassis subsystem reference
  private SwerveModuleState[] states; // the states to put the swerve modules at

  private final double point; // point to turn the wheels to
  /**
   * Creates a new GoToAngle command
   *
   * @param subsystem The subsystem used by this command.
   * @param point the degree to turn the steering wheel to
   */
  public GoToAngle(Chassis subsystem, double point) {
    m_chassis = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.point = point;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_chassis.turnToAngle(point); // turns the wheels to an angle
    SwerveModuleState desiredState = new SwerveModuleState(); // empty state
    // gives the motors an output
    states = new SwerveModuleState[]{desiredState, desiredState, desiredState, desiredState};
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_chassis.setModuleStates(states);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_chassis.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_chassis.turnToAnglePIDIsDone();
  }
}
