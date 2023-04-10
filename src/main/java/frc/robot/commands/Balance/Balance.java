// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Balance;

import frc.robot.sensors.Navx;
import frc.robot.subsystems.Chassis;
import frc.robot.supportingClasses.Auton.AutonCommand;
import frc.robot.supportingClasses.Auton.AutonManager;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;

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
   * The auton manager singleton which is used to generate paths to go to the balancing pad.
   */
  private final AutonManager m_autonManager;

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
   * The distance to drive in meters
   * Should get progressivly smaller as the robot balances.
   */
  protected double distanceToDrive;

  /**
   * Holds the current auton command that we are running.
   * Changes in execute as the robot balances.
   */
  protected AutonCommand autonCommand;

  /**
   * The past april tag value before this command started running.
   * Initialized in init as opposed to constructor.
   */
  protected boolean pastAprilTagValue;

  /**
   * Creates a new Balance command
   *
   * @param chassis The subsystem used by this command.
   */
  public Balance(Chassis chassis, AutonManager manager) {
    m_chassis = chassis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);

    m_timer = new Timer();
    timeToWait =  0.5;

    m_autonManager = manager;
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
    distanceToDrive = 1.5; // default length to drive in meters

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
        Pose2d currentPos = m_chassis.getPose2d();
        autonCommand = m_autonManager.onTheFlyGenerator(currentPos, currentPos.plus(
          new Transform2d(
            new Translation2d(distanceToDrive, 0), new Rotation2d()))
          );
        state = State.DrivingDistance;
        m_timer.stop();
        m_timer.reset();
        autonCommand.initialize();
      }
    } else { // state is equal to DrivingDistance);
      if (autonCommand.isFinished()) {
        autonCommand.end(false);
        state = State.Waiting;
        m_timer.reset();
        m_timer.start();
        distanceToDrive = (distanceToDrive / 2) * -Math.signum(Navx.getPitch()); // multiplies the distance to drive by the sign of our pitch
      } else {
        autonCommand.execute();
      }
    }
  }

  /**
   * Called once the command ends or is interrupted.
   * Restores field orriented to its previous state.
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
