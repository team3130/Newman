// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.supportingClasses.AutonManager;

/** A command to go to whatever origin is */
public class GoToOrigin extends CommandBase {
  private final Chassis m_chassis;
  private final AutonManager autonManager;
  private Thread m_thread;
  private boolean firstHit = true;

  private CommandBase autonCommand;

  /**
   * Creates a new goToOrigin Command.
   *
   * @param chassis The subsystem used by this command.
   */
  public GoToOrigin(Chassis chassis, AutonManager manager) {
    m_chassis = chassis;
    // Use addRequirements() here to declare subsystem dependencies.
    autonManager = manager;
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    firstHit = true;
    autonCommand = null;

    m_thread = new Thread(() -> autonCommand = autonManager.backToStart(m_chassis.getPose2d()));
    m_thread.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (autonCommand != null) {
      if (firstHit) {
          autonCommand.initialize();
          firstHit = false;
      } else {
        autonCommand.execute();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    autonCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (autonCommand != null) {
      return autonCommand.isFinished();
    }
    return false;
  }
}
