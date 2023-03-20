// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Newman_Constants.Constants;
import frc.robot.subsystems.PlacementExtensionArm;
import frc.robot.subsystems.PlacementRotaryArm;

/** An example command that uses an example subsystem. */
public class MoveRotaryArm extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PlacementRotaryArm m_rotaryArm;
  private final PlacementExtensionArm m_extensionArm;

  private final GenericEntry success;

  private double lastSpeed = -1;
  private double lastTorque = -1;

  public Joystick m_xboxController;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveRotaryArm(PlacementRotaryArm subsystem, PlacementExtensionArm extensionArm, Joystick m_xboxController) {
    // specify whether rotary arm should be lowered or raised by setting the direction parameter as either -1 or 1, respectively
    m_rotaryArm = subsystem;
    this.m_xboxController = m_xboxController;
    m_extensionArm = extensionArm;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_rotaryArm);

    success = Shuffleboard.getTab("Test").add("kV", 0).getEntry();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double y = -m_xboxController.getRawAxis(Constants.Buttons.LST_AXS_RJOYSTICKY); // inverted?
    y = y * Math.abs(y);

    if (Math.abs(y) < Constants.kDeadband || (m_rotaryArm.brokeLimit()) && y < 0) {
      y = 0;
    }

    if (m_rotaryArm.pastLimit() && y > 0) {
      y = 0;
    }

/*    if (y == 0) {
      m_rotaryArm.engageBrake();
    }
    else {
      m_rotaryArm.releaseBrake();
    }*/

    m_rotaryArm.rotateRotaryArm(y); //that max is currently bs

    if (Constants.debugMode) {
      middleMan();
    }
  }

  public void middleMan() {
      final double torque = m_rotaryArm.getPositionPlacementArm() * m_extensionArm.getPositionPlacementArm();
      final double currentSpeed = m_rotaryArm.getSpeedPlacementArm();
      if (Math.abs(currentSpeed - lastSpeed) <= 0.025 && Math.abs((currentSpeed / torque) - (lastSpeed / lastTorque)) <= 0.05) {
        success.setDouble(currentSpeed / torque);
      }

      lastTorque = torque;
      lastSpeed = currentSpeed;
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
