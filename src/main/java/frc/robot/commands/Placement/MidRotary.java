// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PlacementExtensionArm;
import frc.robot.subsystems.PlacementRotaryArm;

/** An example command that uses an example subsystem. */
public class MidRotary extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PlacementRotaryArm m_placementRotaryArm;
  private final PlacementExtensionArm m_placementExtensionArm;

  private double positionDeadband = Math.toRadians(2.5);
  private Timer timeRunning = new Timer();

  /**
   * Creates a new ExampleCommand.
   *
   * @param rotary The subsystem used by this command.
   */
  public MidRotary(PlacementRotaryArm rotary, PlacementExtensionArm extension) {
    m_placementRotaryArm = rotary;
    m_placementExtensionArm = extension;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rotary, extension);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //timeRunning.reset();
    m_placementRotaryArm.releaseBrake();
    m_placementRotaryArm.updateValues();
    //timeRunning.start();
    m_placementRotaryArm.makeSetpointMid();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_placementRotaryArm.gotoPos(m_placementExtensionArm.getPositionPlacementArmExtension(),
            m_placementRotaryArm.getPositionPlacementArmAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   // timeRunning.stop();
    // timeRunning.reset();
    m_placementRotaryArm.engageBrake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_placementRotaryArm.isAtPosition();
  }
}
