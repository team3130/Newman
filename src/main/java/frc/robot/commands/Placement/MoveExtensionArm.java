// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtensionArm;
import frc.robot.Newman_Constants.Constants;
import frc.robot.subsystems.RotaryArm;
import frc.robot.supportingClasses.ShuffleboardUpdated;

/** An example command that uses an example subsystem. */
public class MoveExtensionArm extends CommandBase implements ShuffleboardUpdated {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExtensionArm m_extensionArm;
  private final RotaryArm m_rotaryArm;

  public Joystick m_xboxController;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveExtensionArm(ExtensionArm subsystem, RotaryArm rsubsystem, Joystick m_xboxController) {
    m_rotaryArm = rsubsystem;
    m_extensionArm = subsystem;
    this.m_xboxController = m_xboxController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_extensionArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double y = -m_xboxController.getRawAxis(1); // inverted?
    y =y * Math.abs(y);

    if (Math.abs(y) < Constants.kDeadband || (y < 0 && m_extensionArm.hitLimitSwitch())) {
      y = 0;
    }
    if(m_extensionArm.getDistanceExtensionArm()>)
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

  @Override
  public void updateValueFromShuffleboard() {

  }
}
