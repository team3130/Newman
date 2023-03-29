// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement.presets;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtensionArm;

/** A command to go to the mid-scoring position. */
public class GoToPickupWithinBot extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExtensionArm m_extensionArm;

  /**
   * Creates a new GoToMidScoring preset.
   *
   * @param extension The Extension arm subsystem which is used by this command.
   */
  public GoToPickupWithinBot(ExtensionArm extension) {
    m_extensionArm = extension;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(extension);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_extensionArm.extendWithinBot();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_extensionArm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_extensionArm.atPosition();
  }
}
