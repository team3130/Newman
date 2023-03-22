// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement.presets;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtensionArm;
import frc.robot.subsystems.RotaryArm;

/** An example command that uses an example subsystem. */
public class GoToMidScoring extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RotaryArm m_RotaryArm;
  private final ExtensionArm m_ExtensionArm;

  private final double positionDeadband = Math.toRadians(2.5);
  private final Timer timeRunning = new Timer();
  private boolean hasStartedExtended;
  /**
   * Creates a new ExampleCommand.
   *
   * @param rotary The subsystem used by this command.
   */
  public GoToMidScoring(RotaryArm rotary, ExtensionArm extension) {
    m_RotaryArm = rotary;
    m_ExtensionArm = extension;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rotary, extension);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //timeRunning.reset();
    m_RotaryArm.releaseBrake();
    m_RotaryArm.updateValues();
    //timeRunning.start();
    m_RotaryArm.makeSetpointMid();
    hasStartedExtended = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_RotaryArm.gotoPos(m_ExtensionArm.getPositionMeters());

    if(m_RotaryArm.outsideBumper() && !hasStartedExtended) {
      m_ExtensionArm.intermediateArm();
      hasStartedExtended = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   // timeRunning.stop();
    // timeRunning.reset();
    m_RotaryArm.engageBrake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_RotaryArm.isAtPosition();
  }
}
