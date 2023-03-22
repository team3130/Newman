// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement.presets;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtensionArm;
import frc.robot.subsystems.RotaryArm;

/** A preset for scoring. */
public class LowRotary extends CommandBase {
  // the subsystem rotary arm
  private final RotaryArm m_RotaryArm;

  //
  private final ExtensionArm m_ExtensionArm;
  // a timer for how long the command has been running
  private Timer timeRunning = new Timer();

  /**
   * Creates a new ExampleCommand.
   *
   * @param rotary The subsystem used by this command.
   * @param extension not the subsystem of the command, just used for torque
   */
  public LowRotary(RotaryArm rotary, ExtensionArm extension) {
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
    m_RotaryArm.makeSetpointLow();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_RotaryArm.gotoPos(m_ExtensionArm.getPositionMeters());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interupted) {
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
