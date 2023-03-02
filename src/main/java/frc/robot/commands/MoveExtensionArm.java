// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtensionArm;
import frc.robot.Newman_Constants.Constants;
import frc.robot.subsystems.RotaryArm;

/** An example command that uses an example subsystem. */
public class MoveExtensionArm extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExtensionArm m_extensionArm;
  private final RotaryArm m_rotaryArm;
  public XboxController m_xboxController;

  /**
   * Creates a new ExampleCommand.
   *
   * @param extensionArm The extension arm which is the required subsystem.
   * @param rotaryArm the rotary arm subsystem. NOT REQUIRED
   */
  public MoveExtensionArm(ExtensionArm extensionArm, RotaryArm rotaryArm) {
    m_extensionArm = extensionArm;
    m_rotaryArm = rotaryArm;
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
    double rotaryArmAngle = m_rotaryArm.getAngleRotaryArm();
    double extensionArmMaxLength = (Units.inchesToMeters(38) / Math.cos(rotaryArmAngle));
    if ((m_extensionArm.getLengthExtensionArm() < extensionArmMaxLength)) {
      if (m_extensionArm.brokeLimit()) {
        m_extensionArm.stopArm();
      } else if ((m_extensionArm.getLengthExtensionArm() < extensionArmMaxLength) && (m_extensionArm.getLengthExtensionArm() * 1.1 > extensionArmMaxLength)) {
        m_extensionArm.extendExtensionArm(y); //that max is currently bs
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_extensionArm.extendExtensionArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}