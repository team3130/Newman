// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PlacementExtensionArm;
import frc.robot.subsystems.PlacementRotaryArm;

/** An example command that uses an example subsystem. */
public class IntermediateExtension extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PlacementExtensionArm m_placementExtension;
  private final PlacementRotaryArm m_placementRotary;


  /**
   * Creates a new ExampleCommand.
   *
   * @param extension The subsystem used by this command.
   */
  public IntermediateExtension(PlacementExtensionArm extension, PlacementRotaryArm rotary) {
    m_placementExtension = extension;
    m_placementRotary = rotary;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(extension, rotary);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_placementExtension.updateValues();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_placementExtension.outsideBumper(m_placementRotary)) {
      m_placementExtension.intermediateArm();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
