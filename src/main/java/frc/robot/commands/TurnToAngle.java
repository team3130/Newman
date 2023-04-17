// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Chassis;

/** An example command that uses an example subsystem. */
public class TurnToAngle extends InstantCommand {

  /**
   * The subsystem that this command requires
   */
  private final Chassis m_subsystem;
  private final double AngleGoTo;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurnToAngle(Chassis subsystem, double AngleGoTo) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    this.AngleGoTo =AngleGoTo;
  }

  /**
   * Called when the command is initially scheduled.
   */
  @Override
  public void initialize() {
    m_subsystem.SetHoloGoal(AngleGoTo);
  }

}
