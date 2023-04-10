// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

/**
 * A command that uses chassis to spin the wheels to an angle.
 */
public class GoToAngle extends CommandBase {

  /**
   * The chassis singleton which is the subsystem of this command
   */
  private final Chassis m_chassis;

  /**
   * point to turn the wheels to (an angle)
   */
  private final double point;
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

  /**
   * Called once when the command is first scheduled
   */
  @Override
  public void initialize() {
  }

  /**
   * Turns the chassis to an angle based off the passed in point
   */
  @Override
  public void execute() {
    m_chassis.turnToAngle(point); // turns the wheels to an angle
  }

  /**
   * Stops the chassis drive motors
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    m_chassis.stopModules();
  }

  /**
   * @return if chassis turn to angle PID is done
   */
  @Override
  public boolean isFinished() {
    return m_chassis.turnToAnglePIDIsDone();
  }
}
