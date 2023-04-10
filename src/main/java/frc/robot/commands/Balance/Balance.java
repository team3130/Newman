// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Balance;

import frc.robot.sensors.Navx;
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
   * speed to run the drivetrain at in m/s
   */
  protected double speed = 0.3;

  /**
   * The value for if we are off balance
   */
  protected double offBalancePositve = 7;

  /**
   * Stores the state that fieldRelative was at before the command started
   */
  protected boolean previousDriveState;

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
   * Configures the chassis to be robot orriented.
   */
  @Override
  public void initialize() {
    previousDriveState = m_chassis.getFieldRelative();
    m_chassis.setWhetherFieldOriented(false);
  }

  /**
   * Called every time the scheduler runs while the command is scheduled.
   */
  @Override
  public void execute() {
    if (Navx.getPitch() > offBalancePositve) {
      m_chassis.drive(speed, 0, 0);
    }
    else if (Navx.getPitch() < -offBalancePositve) {
      m_chassis.drive(-speed, 0, 0);
    }
    else {
      m_chassis.stopModules();
    }
  }

  /**
   * Called once the command ends or is interrupted.
   * Restores field orriented to its previous state.
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    m_chassis.setWhetherFieldOriented(previousDriveState);
  }

  /**
   * Returns true when the command should end.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
