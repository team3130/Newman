// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement.ManualControl;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Newman_Constants.Constants;
import frc.robot.subsystems.ExtensionArm;
import frc.robot.subsystems.RotaryArm;

/** A command that runs the rotary arm */
public class MoveRotaryArm extends CommandBase {

  /**
   * The singleton for the rotary arm. Is the subsystem that we use.
   */
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RotaryArm m_rotaryArm;

  /**
   * The singleton for the extension arm. Isn't a subsystem but is used to get the length of the arm for gains
   */
  private final ExtensionArm m_extensionArm;

  /**
   * The network table entry for the velocity gain
   */
  private final GenericEntry success;

  /**
   * The actual array which becomes circular
   */
  private final double[] speeds;

  /**
   * The median filter for the velocity gain
   */
  private final MedianFilter filter;

  /**
   * The capacity of the circular array
   */
  private int capacity = -1;

  /**
   * Head for the circular array
   */
  private int head = 0;

  /**
   * The controller that the rotary arm is controller with
   */
  public XboxController m_xboxController;

  /**
   * Creates a new MoveRotaryArm command
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveRotaryArm(RotaryArm subsystem, ExtensionArm extensionArm, XboxController m_xboxController) {
    // specify whether rotary arm should be lowered or raised by setting the direction parameter as either -1 or 1, respectively
    m_rotaryArm = subsystem;
    this.m_xboxController = m_xboxController;
    m_extensionArm = extensionArm;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_rotaryArm);

    speeds = new double[10];

    success = Shuffleboard.getTab("Test").add("rotary kV", 0).getEntry();

    filter = new MedianFilter(7);
  }

  /**
   * Runs once when the scheduler initially schedules the command
   */
  @Override
  public void initialize() {
  }

  /**
   * Runs repeatedly while the command is not interrupted.
   * Controls the rotary arm with the Y axis on the right joystick.
   */
  @Override
  public void execute() {
    double y = -m_xboxController.getRawAxis(Constants.Buttons.LST_AXS_RJOYSTICKY); // inverted?
    y = y * Math.abs(y);

    if (Math.abs(y) < Constants.kDeadband || m_rotaryArm.brakeIsEnabled() ||  (m_rotaryArm.brokeLimit()) && y < 0) {
      y = 0;
    }

    if (m_rotaryArm.pastLimit() && y > 0) {
      y = 0;
    }

    m_rotaryArm.rotateRotaryArm(y); //that max is currently bs
  }

  /**
   * gets the velocity gain?????
   * FUNNY CIRCULAR ARRAY
   */
  public void middleMan(double main) {
      final double torque = Math.sin(m_rotaryArm.getPositionPlacementArmAngle()) * m_extensionArm.getPositionMeters();
      final double currentSpeed = m_rotaryArm.getSpeedPlacementArm();

      if (head + speeds.length == capacity - 1) {
        boolean worked = true;
        for (int i = head + 1; i != capacity; i++) {
          double lastSpeed = speeds[(i - 1) % speeds.length];
          if (!(Math.abs(speeds[(i) % speeds.length] - lastSpeed) <= 0.0075)) {
            worked = false;
            break;
          }
        }
        if (worked) {
          success.setDouble(filter.calculate(main - (Constants.Extension.kRotaryStaticGain * torque) / (currentSpeed)));
        }
        head++;
      }
      speeds[(++capacity% speeds.length)] = currentSpeed;
  }


  /**
   * stop the rotary arm at the end of the command.
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    m_rotaryArm.rotateRotaryArm(0);
  }

  /**
   * @return False. This command does not end.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
