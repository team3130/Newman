// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Balance;

import frc.robot.sensors.Navx;
import frc.robot.subsystems.Chassis;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Balance extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Chassis m_chassis;
  private final double pitchZero = -8.211; //should go in Constants
  private final double pitchDeadband = 5;
  private final double pitchVelocityDeadband = 0.5;


  private double direction;
  private int iterator;
  private boolean pitchVelocityCheck = false;
  private final double driveVelocity = 0.625;
  private double oddPitch;
  private double pitch;
  private double roll;
  private double tilt;
  private double velocityPitch;
  private double velocityRoll;
  private double VelocityTilt;
  private double AccelerationTilt;
  private MedianFilter fVelocityTilt;
  private double prev;
  private static ShuffleboardTab tab = Shuffleboard.getTab("Chassis");


  /**
   * speed to run the drivetrain at in m/s
   */
  protected double speed = 0.3;

  /**
   * The value for if we are off balance
   */
  protected double offBalancePositve = 7;

  /**
   * The state of the commnand.
   * 0 = driving distance
   * 1 = waiting for the balancing pad to adjust
   */
  protected int state = 0;

  /**
   * The timer used to wait for the balancing pad to adjust
   */
  protected Timer m_timer;

  /**
   * The timeout for balancing pad
   */
  protected double timeToWait = 0.5;

  /**
   * The distance to drive in meters
   * Should get progressivly smaller as the robot balances.
   */
  protected double distanceToDrive;

  /**
   * Creates a new Balance command
   *
   * @param chassis The subsystem used by this command.
   */
  public Balance(Chassis chassis) {
    m_chassis = chassis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_chassis);
    
  }

  /**
   * Called when the command is initially scheduled.
   * Configures the chassis to be robot orriented.
   */
  @Override
  public void initialize() {
    
  }

  /**
   * Called every time the scheduler runs while the command is scheduled.
   */
  @Override
  public void execute() {
    if (Navx.getPitch() > offBalancePositve) {
      m_chassis.drive(speed, 0, 0);
    }
    else if (Navx.getPitch() < -offBalancePositve) {
      m_chassis.drive(-speed, 0, 0);
    }
    else {
      m_chassis.stopModules();
    }
  }

  /**
   * Called once the command ends or is interrupted.
   * Restores field orriented to its previous state.
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    m_chassis.setWhetherFieldOriented(previousDriveState);
    m_chassis.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Navx.getPitch() < offBalancePositve && Navx.getPitch() > -offBalancePositve;
  }
}
