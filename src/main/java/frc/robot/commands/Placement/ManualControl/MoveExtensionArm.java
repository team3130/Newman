// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement.ManualControl;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Newman_Constants.Constants;
import frc.robot.subsystems.ExtensionArm;

/** A command to move the extension arm based off the joysticks */
public class MoveExtensionArm extends CommandBase {
  private final ExtensionArm m_extensionArm;
  public Joystick m_xboxController;

  /*public boolean justHitLimit = false;*/

  /**
   * Creates a new Move Extension Arm command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveExtensionArm(ExtensionArm subsystem, Joystick m_xboxController) {
    m_extensionArm = subsystem;
    this.m_xboxController = m_xboxController;
//    boolean flag = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_extensionArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double y = -m_xboxController.getRawAxis(Constants.Buttons.LST_AXS_LJOYSTICKY); // inverted
    y = y * Math.abs(y);

    m_extensionArm.spinExtensionArm(y); //that max is currently bs
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_extensionArm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
