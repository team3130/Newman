// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement;

import edu.wpi.first.networktables.GenericEntry;
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

  private final double[] speeds;
  private final double[] torques;

  private int capacity = -1;
  private int head = 0;

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

    speeds = new double[10];
    torques = new double[10];

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

    m_rotaryArm.rotateRotaryArm(y); //that max is currently bs

    if (Constants.debugMode && y > 0) {
      middleMan(y);
    }
  }

  /**
   * gets the velocity gain?????
   */
  public void middleMan(double main) {
      final double torque = Math.sin(m_rotaryArm.getPositionPlacementArmAngle()) * m_extensionArm.getPositionPlacementArmExtension();
      final double currentSpeed = m_rotaryArm.getSpeedPlacementArm();

      if (head + speeds.length == capacity - 1) {
        boolean worked = true;
        for (int i = head + 1; i != capacity; i++) {
          double lastSpeed = speeds[(i - 1) % speeds.length];
          double lastTorque = torques[(i - 1) % speeds.length];
          if (!(
                  Math.abs(speeds[(i) % speeds.length] - lastSpeed) <= 0.025 &&
                          Math.abs((speeds[i % speeds.length] / torques[i % speeds.length]) - (lastSpeed / lastTorque)) <= 0.05)) {
            worked = false;
            break;
          }
        }
        if (worked) {
          success.setDouble(main / (torque * currentSpeed));
        }
        speeds[(++capacity % speeds.length)] = currentSpeed;
        torques[(capacity% speeds.length)] = torque;
        head++;
      }
      else {
        speeds[(++capacity% speeds.length)] = currentSpeed;
        torques[(capacity % speeds.length)] = torque;
      }
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
