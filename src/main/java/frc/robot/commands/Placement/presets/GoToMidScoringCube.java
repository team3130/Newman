// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement.presets;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtensionArm;
import frc.robot.subsystems.RotaryArm;

/** A command to go to the mid-scoring position. */
public class GoToMidScoringCube extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RotaryArm m_rotaryArm;
  private final ExtensionArm m_extensionArm;

  private boolean hasStartedExtended = false;

  /**
   * Creates a new GoToMidScoringCube preset.
   *
   * @param rotary The subsystem used by this command.
   */
  public GoToMidScoringCube(RotaryArm rotary, ExtensionArm extension) {
    m_rotaryArm = rotary;
    m_extensionArm = extension;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rotary, extension);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     m_rotaryArm.releaseBrake();
     m_rotaryArm.makeSetpointMid();
     hasStartedExtended = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     m_rotaryArm.gotoPos(m_extensionArm.getPositionMeters());

     if (!hasStartedExtended && m_rotaryArm.outsideBumper()) { // may need way outside bumper
       m_extensionArm.intermediateArm();
       hasStartedExtended = true;
     }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_rotaryArm.engageBrake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_rotaryArm.isAtPosition() && m_extensionArm.atPosition();
  }
}
