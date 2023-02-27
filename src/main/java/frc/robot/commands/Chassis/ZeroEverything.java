// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ExampleSubsystem;

/** A command to zero the heading and encoders. */
public class ZeroEverything extends InstantCommand {
  private final Chassis m_subsystem; // chassis subsystem

  /**
   * Creates a new ZeroEverything command in order to reset navx heading and the encoders
   *
   * @param subsystem The subsystem used by this command.
   */
  public ZeroEverything(Chassis subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d()));
  }
}
