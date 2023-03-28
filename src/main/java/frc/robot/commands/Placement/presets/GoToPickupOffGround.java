// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement.presets;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtensionArm;
import frc.robot.subsystems.RotaryArm;


/** A preset command to go to the high position for the rotary arm */
public class GoToPickupOffGround extends CommandBase {
  // required subsystem
  private final RotaryArm m_rotaryArm;
  private final ExtensionArm m_extensionArm;

  private boolean hasStartedExtended;

  /**
   * Creates a new command that runs the preset to get to high rotary
   *
   * @param rotary The rotary subsystem which this command requires
   * @param extension the extension arm subsystem which this command requires
   */
  public GoToPickupOffGround(RotaryArm rotary, ExtensionArm extension) {
    m_rotaryArm = rotary;
    m_extensionArm = extension;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rotary, extension);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_rotaryArm.releaseBrake();
    m_rotaryArm.makeSetpointGroundCone();
    hasStartedExtended = false;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_rotaryArm.gotoPos(m_extensionArm.getPositionMeters());

    if (m_rotaryArm.isAtPosition() && !hasStartedExtended) { // may need way outside bumper
      m_extensionArm.extendArmToGround();
      hasStartedExtended = true;
    }
  }

    // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_rotaryArm.stop();
    m_extensionArm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_rotaryArm.isAtPosition()  && m_extensionArm.atPosition();
  }
}
