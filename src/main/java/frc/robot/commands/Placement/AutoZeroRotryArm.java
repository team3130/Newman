// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RotaryArm;

/** An example command that uses an example subsystem. */
public class AutoZeroRotryArm extends CommandBase {
  private final RotaryArm m_RotaryArm;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoZeroRotryArm(RotaryArm subsystem) {
    m_RotaryArm = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_RotaryArm.releaseBrake();
    m_RotaryArm.spin(-0.1);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interupted) {
    m_RotaryArm.engageBrake();
    m_RotaryArm.resetEncoder();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_RotaryArm.brokeLimit();
  }
}
