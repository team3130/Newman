// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeBeaterBar;

/** A command to toggle the intake beater bar that uses Intake as the subsystem. */
public class IntakeToggleBeaterBar extends InstantCommand {
  private final IntakeBeaterBar m_beaterBar;

  /**
   * Creates a new IntakeToggleBeaterBar command
   *
   * @param beaterBar The subsystem used by this command.
   */
  public IntakeToggleBeaterBar(IntakeBeaterBar beaterBar) {
    m_beaterBar = beaterBar;;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_beaterBar);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!m_beaterBar.isSpinning()) {
      m_beaterBar.spin();
    }
    else {
      m_beaterBar.stop();
    }
  }
}
