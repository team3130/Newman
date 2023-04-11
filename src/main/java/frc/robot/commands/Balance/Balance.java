// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Balance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.Navx;
import frc.robot.subsystems.Chassis;
import frc.robot.supportingClasses.Auton.AutonManager;

/** 
 * A command that balances the robot on the balancing pad.
 */
public class Balance extends CommandBase {

  /**
   * the chassis singleton which is required by this subsystem
   */
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Chassis m_chassis;

  /**
   * The state of the commnand.
   * Driving distance is while the command is driving a certain distance.
   * Waiting is while the command is waiting for the balancing pad to adjust and whil waiting generate new trajcetorys.
   */
  protected enum State {
    DrivingDistance, Waiting
  }

  /**
   * Holds what state we are in right now.
   */
  protected State state;

  /**
   * The timer used to wait for the balancing pad to adjust
   */
  protected final Timer m_timer;

  /**
   * The timeout for balancing pad.
   */
  protected final double timeToWait;

  /**
   * Holds the distance to drive the wheels to.
   */
  protected double driveDistance;

  /**
   * The speed to run the drivetrain at.
   */
  protected final double speed = 0.3;

  /**
   * The past april tag value before this command started running.
   * Initialized in init as opposed to constructor.
   */
  protected boolean pastAprilTagValue;

  /**
   * The current set point we are going to.
   * Needs to be combined with coming from to make sure you are on the opposite side of the set point than
   * where you are coming from.
   */
  protected double currentSetpoint;

  /**
   * The spot we are coming from.
   */
  protected double comingFrom;

  /**
   * Creates a new Balance command
   *
   * @param chassis The subsystem used by this command.
   */
  public Balance(Chassis chassis) {
    m_chassis = chassis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);

    m_timer = new Timer();
    timeToWait =  0.5;
  }

  /**
   * Called when the command is initially scheduled.
   * Configures the chassis to be robot orriented.
   */
  @Override
  public void initialize() {
    // store what april tag usage we had before this command started
    pastAprilTagValue = m_chassis.getUsingAprilTags();
    m_chassis.setAprilTagUsage(false);

    state = State.Waiting; // the default state to be at when the command starts
    driveDistance = 1.5; // default length to drive in meters

    m_timer.reset();
    m_timer.start();
  }

  /**
   * Called every time the scheduler runs while the command is scheduled.
   */
  @Override
  public void execute() {
    if (state == State.Waiting) {
      if (m_timer.hasElapsed(timeToWait)) {
        double currentPos = m_chassis.getPose2d().getX();
        comingFrom = currentPos;
        driveDistance = (driveDistance / 2) * Math.signum(Navx.getPitch()); // multiplies the distance to drive by the sign of our pitch
        currentSetpoint = currentPos + driveDistance;
        m_chassis.drive(speed * Math.signum(driveDistance), 0, 0, true);
        state = State.DrivingDistance;
        m_timer.stop();
        m_timer.reset();
      }
    } else { // state is equal to DrivingDistance
      if (m_chassis.getPose2d().getX() - currentSetpoint >= 0) {
        m_chassis.stopModules();
        state = State.Waiting;
        m_timer.reset();
        m_timer.start();
      }
    }
  }

  /**
   * Called once the command ends or is interrupted.
   * Restores field oriented to its previous state.
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    m_chassis.stopModules();
    m_chassis.setAprilTagUsage(pastAprilTagValue);

    m_timer.stop();
    m_timer.reset();
  }

  /**
   * is finished if we are balanced
   */
  @Override
  public boolean isFinished() {
    return Math.abs(Navx.getPitch()) <= 3;
  }
}
