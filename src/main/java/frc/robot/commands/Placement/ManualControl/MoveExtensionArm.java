// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement.ManualControl;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Newman_Constants.Constants;
import frc.robot.subsystems.ExtensionArm;

/**
 * A command to move the extension arm based off the joysticks
 */
public class MoveExtensionArm extends CommandBase {

  /**
   * The singleton for the extension arm. Is required by this subsystem
   */
  private final ExtensionArm m_extensionArm;

  /**
   * The xbox controller that the extension arm should be controlled from
   */
  public XboxController m_xboxController;

  /**
   * Creates a new MoveExtensionArm command.
   *
   * @param extension The subsystem used by this command.
   * @param m_xboxController the controller that this subsystem uses (should be weapons)
   */
  public MoveExtensionArm(ExtensionArm extension, XboxController m_xboxController) {
    m_extensionArm = extension;
    this.m_xboxController = m_xboxController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_extensionArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * Reads the controllers input and then runs the extension arm.
   * Soft limits are checked when {@link ExtensionArm#spinExtensionArm(double)} is called.
   */
  @Override
  public void execute() {
    double y = -m_xboxController.getRawAxis(Constants.Buttons.LST_AXS_LJOYSTICKY); // inverted
    y = y * Math.abs(y);

    m_extensionArm.spinExtensionArm(y); //that max is currently bs
  }

  /**
   * Stops the extension arm
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    m_extensionArm.stop();
  }

  /**
   * @return False. meant to be a default command so it never ends
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
