// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement.presets;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtensionArm;
import frc.robot.subsystems.RotaryArm;


/** A preset command to go to the high position for the rotary arm */
public class GoToHighSchoring extends CommandBase {
  // required subsystem
  private final RotaryArm m_rotaryArm;
  private final ExtensionArm m_extensionArm;
  private final Timer timeRunning = new Timer();

  private boolean hasStartedExtended;

  /**
   * Creates a new command that runs the preset to get to high rotary
   *
   * @param rotary The rotary subsystem which this command requires
   * @param extension the extension arm subsystem which this command requires
   */
  public GoToHighSchoring(RotaryArm rotary, ExtensionArm extension) {
    m_rotaryArm = rotary;
    m_extensionArm = extension;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rotary, extension);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // timeRunning.reset();
    m_rotaryArm.releaseBrake();
    m_rotaryArm.updateValues();
  //  timeRunning.start();
    m_rotaryArm.makeSetpointHigh();
    hasStartedExtended = false;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_rotaryArm.gotoPos(m_extensionArm.getPositionMeters());

    if (m_rotaryArm.outsideBumper() && !hasStartedExtended) { //may need way outside bumper
      m_extensionArm.extendArmFull();
      hasStartedExtended = true;
    }
  }

    // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_rotaryArm.stop();
    m_rotaryArm.engageBrake();
    m_extensionArm.stop();
  //  timeRunning.stop();
   // timeRunning.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_rotaryArm.isAtPosition();
  }
}
