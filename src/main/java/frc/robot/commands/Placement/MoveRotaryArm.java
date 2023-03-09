// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Newman_Constants.Constants;
import frc.robot.subsystems.RotaryArm;

/** An example command that uses an example subsystem. */
public class MoveRotaryArm extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RotaryArm m_rotaryArm;

  public Joystick m_xboxController;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveRotaryArm(RotaryArm subsystem, Joystick m_xboxController) {
    // specify whether rotary arm should be lowered or raised by setting the direction parameter as either -1 or 1, respectively
    m_rotaryArm = subsystem;
    this.m_xboxController = m_xboxController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_rotaryArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double y = m_xboxController.getRawAxis(Constants.Buttons.LST_AXS_RJOYSTICKY); // inverted?
    y *= Math.abs(y);

    if (Math.abs(y) < Constants.kDeadband) {
      y = 0;
    }
    m_rotaryArm.rotateRotaryArm(y); //that max is currently bs
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_rotaryArm.rotateRotaryArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
