// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtensionArm;


/**
 * A command to zero the extension arm stops when it hits the limit switch uses extension
 */
public class AutoZeroExtensionArm extends CommandBase {
  /**
   * The extension arm singleton which is required by this command.
   */
  private final ExtensionArm m_extensionArm;

  /**
   * Creates a new AutoZeroExtensionArm.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoZeroExtensionArm(ExtensionArm subsystem) {
    m_extensionArm = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_extensionArm);
  }

  /**
   * Called when the command is initially scheduled.
   * Sets the extension arm to rewind as fast as possible
   */
  @Override
  public void initialize() {
    m_extensionArm.spinExtensionArm(-1);
  }

  /**
   * Called every time the scheduler runs while the command is scheduled.
   * Does nothing.
   */
  @Override
  public void execute() {}

  /**
   * Stops the extension arm.
   * If the command wasn't interrupted it will also reset the extension arms encoders to 0
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    m_extensionArm.stop();
    if (!interrupted) {
      m_extensionArm.resetEncoders();
    }

  }

  /**
   * @return true when the limit switch is hit. See {@link ExtensionArm#brokeLimit()}
   */
  @Override
  public boolean isFinished() {
    return m_extensionArm.brokeLimit();
  }

}
