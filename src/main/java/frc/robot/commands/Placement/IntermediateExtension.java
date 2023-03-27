// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtensionArm;
import frc.robot.subsystems.RotaryArm;

/** An example command that uses an example subsystem. */
public class IntermediateExtension extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExtensionArm m_extension;
  private final RotaryArm m_rotary;
  private boolean ran = false;


  /**
   * Creates a new Intermediate Extension command
   *
   * @param extension The subsystem that this command requires
   * @param rotary the rotary subsystem
   */
  public IntermediateExtension(ExtensionArm extension, RotaryArm rotary) {
    m_extension = extension;
    m_rotary = rotary;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(extension);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_extension.updateValues();
    ran = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!ran && m_rotary.outsideBumper()) {
      m_extension.intermediateArm();
      ran = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_extension.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_extension.atPosition();
  }
}
