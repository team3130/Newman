// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PlacementExtensionArm;
import frc.robot.subsystems.PlacementRotaryArm;

/** An example command that uses an example subsystem. */
public class IntermediatePlacement extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PlacementExtensionArm m_placement;
  private final PlacementRotaryArm m_placementRotary;

  private boolean ran = false;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntermediatePlacement(PlacementExtensionArm subsystem, PlacementRotaryArm rotary) {
    m_placement = subsystem;
    m_placementRotary = rotary;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    addRequirements(rotary);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_placement.updateValues();
    ran = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_placementRotary.getPositionPlacementArm()>Math.toRadians(30) & !ran){
      m_placement.collapseArm();
      ran = true;
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
