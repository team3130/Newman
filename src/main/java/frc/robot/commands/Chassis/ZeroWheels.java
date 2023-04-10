// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

/** A command to zero wheels of chassis */
public class ZeroWheels extends CommandBase {

  /**
   * The chassis singleton which is the subsystem for this command
   */
  private final Chassis m_chassis;

  /**
   * Creates a new ZeroWheels
   *
   * @param chassis The subsystem used by this command.
   */
  public ZeroWheels(Chassis chassis) {
    m_chassis = chassis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
  }

  /**
   * Called when the scheduler starts the command
   */
  @Override
  public void initialize() {
  }

  /**
   * Sets the angle PID controller to 0 degrees and calculates output for the motors
   */
  @Override
  public void execute() {
    m_chassis.turnToAngle(0);
  }

  /**
   * Called when the scheduler ends the command.
   * Stops the chassis motors
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    m_chassis.stopModules();
  }

  /**
   * @return if the wheel PID controller is done. AKA are wheels zero-ed
   */
  @Override
  public boolean isFinished() {
    return m_chassis.turnToAnglePIDIsDone();
  }
}
