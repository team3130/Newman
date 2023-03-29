// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExtensionArm;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.supportingClasses.BoundingBox;
import frc.robot.subsystems.Chassis;

/** An example command that uses an example subsystem. */
public class HoldPos extends CommandBase {
  private final ExtensionArm m_extensionarm;

  /*
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  // lmp stands for low, mid, or high
  public HoldPos(ExtensionArm extensionarm, int lmp) {
    m_extensionarm = extensionarm;
    addRequirements(extensionarm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (BoundingBox.boxBad(m_extensionarm.armPos) && m_extensionarm.getLengthExtensionArm() + 0.09525) {
      //arm stop
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
