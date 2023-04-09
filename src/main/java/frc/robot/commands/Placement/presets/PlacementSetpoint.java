// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement.presets;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtensionArm;
import frc.robot.subsystems.RotaryArm;


/** A preset command to go to the high position for the rotary arm */
public abstract class PlacementSetpoint extends CommandBase {

  /**
   * The rotary arm singleton which is a subsystem that is required by this command.
   */
  private final RotaryArm m_rotaryArm;

  /**
   * The subsystem for the extension arm.
   * Is required by this command
   */
  private final ExtensionArm m_extensionArm;

  /**
   * Have we started extending the extension arm
   */
  private boolean hasStartedExtended;

  /**
   * The angle (radians) to move the rotary arm to.
   */
  protected final double angleToGoTo;

  /**
   * The length in ticks that the extension arm is supposed to go to.
   */
  protected final double lengthForExtensionArm;

  /**
   * The angle (radians) that the rotary arm is supposed to get to before we start extending the extension arm.
   * Bumper is about 30 degrees.
   */
  protected final double angleBeforeExtension;

  /**
   * If this command is supposed to use the extension arm or not
   */
  protected final boolean usesExtensionArm;

  /**
   * Creates a new command that outlines the preset for just the rotary arm.
   * Extension arm is still needed for feedforward calculation but will not be required by this command.
   *
   * @param rotary the rotary subsystem. This gets required.
   * @param extensionArm the subsystem for the extension arm. This gets required.
   * @param angle (radians) the angle in radians to go to
   */
  public PlacementSetpoint(RotaryArm rotary, ExtensionArm extensionArm, double angle) {
    m_rotaryArm = rotary;
    m_extensionArm = extensionArm;
    addRequirements(rotary);

    this.angleToGoTo = angle;
    this.lengthForExtensionArm = 0;
    this.angleBeforeExtension = 0;

    usesExtensionArm = false;
  }

  /**
   * Creates a new command that outlines the preset for the rotary AND extension arm.
   *
   * @param rotary The rotary subsystem which this command requires
   * @param extension the extension arm subsystem which this command requires
   * @param angleToGoTo the angle (radians) to move the rotary arm to in radians
   * @param lengthToGoTo the length (ticks) for the extension arm to go to
   * @param angleBeforeExtension the angle (degrees) that we want to reach before extending the extension arm
   */
  public PlacementSetpoint(RotaryArm rotary, ExtensionArm extension,
                           double angleToGoTo, double lengthToGoTo, double angleBeforeExtension) {
    m_rotaryArm = rotary;
    m_extensionArm = extension;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rotary, extension);

    // setpoints
    this.angleToGoTo = angleToGoTo;
    this.lengthForExtensionArm = lengthToGoTo;
    // tolerance
    this.angleBeforeExtension = Math.toRadians(angleBeforeExtension);

    usesExtensionArm = true;
  }

  /**
   * Called when the preset is initially scheduled to wpilib.
   * Sets the rotary arm setpoint and releases the brake.
   */
  @Override
  public void initialize() {
     m_rotaryArm.releaseBrake();
     m_rotaryArm.makeSetpoint(angleToGoTo);
     hasStartedExtended = false;
  }

  /**
   * Extends the extension arm if it is past {@link #angleBeforeExtension}. updates the rotary arm PID controller.
   */
  @Override
  public void execute() {
     m_rotaryArm.gotoPos(m_extensionArm.getPositionTicks());

     if (usesExtensionArm && !hasStartedExtended && m_rotaryArm.pastAngle(angleToGoTo)) {
        m_extensionArm.extendArmTo(lengthForExtensionArm);
        hasStartedExtended = true;
     }
  }

  /**
   * Called once the command ends or is interrupted.
   * Stops the rotary and extension arms and engages the brake.
   */
  @Override
  public void end(boolean interrupted) {
    m_rotaryArm.stop();
    m_rotaryArm.engageBrake();
    if (usesExtensionArm) {
      m_extensionArm.stop();
    }
  }

  /**
   * @return if both the rotary and extension arm are at their setpoints and
   *            the extension arm has had a chance to start extending if we are supposed to be using the extension arm.
   */
  @Override
  public boolean isFinished() {
    return hasStartedExtended && m_rotaryArm.isAtPosition() && (!usesExtensionArm || m_extensionArm.atPosition());
  }
}
