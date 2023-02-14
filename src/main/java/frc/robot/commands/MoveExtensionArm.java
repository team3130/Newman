// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtensionArm;
import frc.robot.Newman_Constants.Constants;

/** An example command that uses an example subsystem. */
public class MoveExtensionArm extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExtensionArm m_extensionArm;
  private int dir;
  public XboxController m_xboxController;
  public static double extensionArmMaxSpeed = 1; //TODO make this an actual number & add it to constants

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveExtensionArm(ExtensionArm subsystem, int direction/*extend or retract, 1 or -1, respectively*/) {
    dir = direction;
    m_extensionArm = subsystem;
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
    double y = m_xboxController.getRawAxis(0); // inverted?

    if (Math.abs(y) < Constants.kDeadband) {
      y = 0;
    }
    m_extensionArm.ExtendExtensionArm(y * extensionArmMaxSpeed); //that max is currently bs

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_extensionArm.ExtendExtensionArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
