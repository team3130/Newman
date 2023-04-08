// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

/**
 * An Instant command that toggles the intake between out and in for HP station.
 * Requires the {@link Intake} subsystem
 */
public class ToggleIntake extends InstantCommand {

  /**
   * The Intake singleton which is required by this command
   */
  private final Intake m_intake;

  /*
   * Creates a new ToggleIntake command.
   * @param intake
   */
  public ToggleIntake(Intake intake) {
    m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  /**
   * Gets called once by the scheduler when the command is first scheduled.
   * Toggles the intake.
   */
  @Override
  public void initialize() {
    m_intake.toggle();
  }


}
