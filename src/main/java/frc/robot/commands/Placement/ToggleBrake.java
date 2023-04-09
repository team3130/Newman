// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RotaryArm;

/**
 * An Instant command that toggles the solenoid that controls the brake piston
 * */
public class ToggleBrake extends InstantCommand {
  /**
   * The rotary arm singleton.
   * It is required by this command.
   */
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RotaryArm m_rotary;

  /**
   * Creates a new ToggleBrake command
   *
   * @param rotary The RotaryArm subsystem which used by this command.
   */
  public ToggleBrake(RotaryArm rotary) {
    m_rotary = rotary;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rotary);
  }

  /**
   * Toggles the brake on rotary arm
   */
  @Override
  public void initialize() {
    m_rotary.toggleBrake();
  }

}
