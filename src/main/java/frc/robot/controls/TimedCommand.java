// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A command that will run for a certain amount of time.
 * You can either use this command raw if you want to wait a certain amount of time,
 * or you can subclass this and call super.isFinished().
 */
public class TimedCommand extends CommandBase {

  /**
   * The time that the command will run for
   */
  private final double time;

  /**
   * The timer that the command uses to keep track of how much time has passed
   */
  private final Timer m_timer;

  /*
   * Creates a new TimedCommand
   *
   * @param subsystem The subsystem used by this command.
   */
  public TimedCommand(double time) {
    this.time = time;
    m_timer = new Timer();
  }

  /**
   * Resets the timer to 0.
   * Starts the timer.
   */
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  /**
   * Does nothing
   */
  @Override
  public void execute() {}

  /**
   * Stops the timer and resets it.
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_timer.reset();
  }

  /**
   * @return whether the specified time has passed since the command was scheduled.
   */
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(time);
  }
}
