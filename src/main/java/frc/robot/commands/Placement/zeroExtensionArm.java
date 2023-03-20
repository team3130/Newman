// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Newman_Constants.Constants;
import frc.robot.subsystems.PlacementExtensionArm;
import frc.robot.subsystems.PlacementRotaryArm;



/** A command to zero the extension arm stops when it hits the limit switch uses extension */
public class zeroExtensionArm extends CommandBase {
  private final PlacementExtensionArm m_extensionArm;

  /**
   * Creates a new zeroExtensionArm.
   *
   * @param subsystem The subsystem used by this command.
   */
  public zeroExtensionArm(PlacementExtensionArm subsystem) {
    m_extensionArm = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_extensionArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_extensionArm.spinExtensionArm(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_extensionArm.stop();
    if (Math.abs(m_extensionArm.getPositionPlacementArm()) >(Constants.kMaxExtensionLength * 1.25)) {
      m_extensionArm.setSign(-1);
    }
    m_extensionArm.resetEncoders();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_extensionArm.brokeLimit();
  }

}
