// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Balance;

import frc.robot.subsystems.Chassis;
import frc.robot.supportingClasses.Auton.AutonCommand;
import frc.robot.supportingClasses.Auton.AutonManager;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** 
 * A command to automatically balance the bot
 */
public class Balance extends CommandBase {

  /**
   * The chassis singleton which is required by this command.
   */
  private final Chassis m_chassis;

  /**
   * Should be 1 or negative 1.
   * Gets multiplied by how hard we should drive in order to determine where we should go to.
   */
  protected int sign = 1;

  /**
   * Creates a new Balance command
   *
   * @param chassis the chassis subsystem.
   */
  public Balance(Chassis chassis) {
    m_chassis = chassis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
  }

  /**
   * Called when the command is initially scheduled.
   */
  @Override
  public void initialize() {

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
