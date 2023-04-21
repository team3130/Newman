// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RotaryArm;

/**
 * A command that zeroes the rotary arm automatically
 */
public class AutoZeroRotryArm extends CommandBase {
  /**
   * The rotary arm singleton which is required by this command
   */
  private final RotaryArm m_rotaryArm;

  /**
   * Creates a new AutoZeroRotaryArm
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoZeroRotryArm(RotaryArm subsystem) {
    m_rotaryArm = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  /**
   * Called when the command is initially scheduled.
   * Spins the rotary arm backwards at -15% power. (15% was an arbitrary value that looked: "not too sketchy")
   */
  @Override
  public void initialize() {
     m_rotaryArm.releaseBrake();
     m_rotaryArm.spin(-0.25);

  }

  /**
   * Called every time the scheduler runs while the command is scheduled.
   * Does nothing.
   */
  @Override
  public void execute() {}

  /**
   * Called once the command ends or is interrupted.
   * Engages the brake and stops any percent output to the motor.
   * If the command wasn't interrupted then it is at the limit switch, and we can reset the rotary arms encoders.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    m_rotaryArm.engageBrake();
    m_rotaryArm.stop();
    if (!interrupted) {
      m_rotaryArm.resetEncoder();
    }
  }

  /**
   * @return if the limit switch for the rotary arm is broken
   */
  @Override
  public boolean isFinished() {
    return m_rotaryArm.brokeLimit();
  }
}
