// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A command that does nothing and is always finished
 */
public class DoNothing extends CommandBase {

  /*
   * Creates a new DoNothing command
   */
  public DoNothing() {}

  /**
   * Does nothing
   */
  @Override
  public void initialize() {
  }

  /**
   * Does nothing
   */
  @Override
  public void execute() {}

  /**
   * Does nothing
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {}

  /**
   * Is always done
   */
  @Override
  public boolean isFinished() {
    return true;
  }
}
