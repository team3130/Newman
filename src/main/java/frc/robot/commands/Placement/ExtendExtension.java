// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement;

import com.ctre.phoenix.schedulers.ConcurrentScheduler;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Newman_Constants.Constants;
import frc.robot.subsystems.ExtensionArm;

/** An example command that uses an example subsystem. */
public class ExtendExtension extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExtensionArm m_placementExtension;

  /**
   * Creates a new Extend Extension command
   *
   * @param extension The subsystem used by this command.
   */
  public ExtendExtension(ExtensionArm extension) {
    m_placementExtension = extension;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(extension);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Constants.debugMode) {
      m_placementExtension.updateValues();
    }
    m_placementExtension.extendArmFull();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_placementExtension.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
