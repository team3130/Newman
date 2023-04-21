// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Newman_Constants.Constants;
import frc.robot.sensors.Navx;

import java.sql.Driver;

public class Robot extends TimedRobot {

  private Alliance alliance;
  private short counter = 0;

  /**
   * A timer for resetting odometry. Gets started on robotInit and will run until we reset odometry with april tags
   */
  private Timer timer;

  /**
   * The robot container singleton which declares button bindings and initializes subsystems
   */
  private RobotContainer m_robotContainer;

  /**
   * Whether we have reset odometry without april tags yet
   */
  private boolean haveResetManually = false;
  
  private boolean suceededTeleReset = false;

  /**
   * Initializes robot container and the timer for resetting odometry.
   */
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    timer = new Timer();
    timer.reset();
    timer.start();
    alliance = Alliance.Invalid;
  }

  /**
   * Gets ran every loop while code is running on the roborio.
   * Runs the command scheduler and {@link RobotContainer#periodic()}.
   * Has logic to reset odometry based off of april tags after a certain amount of time to allow
   * for the absolute encoders to start up. If we can't see april tags then we reset to 0, 0.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.periodic();
    if (timer.hasElapsed(Constants.kResetTime)) {
      if (Constants.useAprilTags) {
        if (!haveResetManually) {
          m_robotContainer.resetOdometryWithoutApril();
          haveResetManually = true;
        } else {
          if (m_robotContainer.resetOdometryWithAprilTag()) {
            timer.stop();
            timer.reset();
          }
        }
      } else {
        m_robotContainer.resetOdometryWithoutApril();
        timer.stop();
        timer.reset();
      }
    }
    else {
      m_robotContainer.updateChassisPose();
    }
    if (alliance == Alliance.Invalid) {
      alliance = DriverStation.getAlliance();
    }
    else if (counter > 100) {
      if (DriverStation.getAlliance() != alliance) {
        alliance = DriverStation.getAlliance();
        m_robotContainer.getLimelight().modifyToAllianceStation(alliance);
      }
      counter = -1;
    }
    counter++;
  }

    @Override
    public void disabledInit () {
    }

    @Override
    public void disabledPeriodic () {
    }

    @Override
    public void disabledExit () {
    }

  /**
   * Gets ran when we enable in auton.
   * Schedules the auton command that is selected on network tables. Default is DoNothing
   */
  @Override
    public void autonomousInit () {
      CommandScheduler.getInstance().cancelAll();
      CommandScheduler.getInstance().schedule(m_robotContainer.packageAuton(m_robotContainer.getAutonCmd()));
      Navx.setPitchZero(Navx.getPitch());
    }

    @Override
    public void autonomousPeriodic () {
    }

    @Override
    public void autonomousExit () {
      timer.reset();
      timer.start();
    }

  /**
   * Gets ran when we start teleop / when we enable in teleop.
   * Zeroes the rotary and extension arms and unclamps the manipulator.
   */
  @Override
    public void teleopInit () {
      CommandScheduler.getInstance().cancelAll();
      
      // zero the rotary arm into frame perimeter for both safety and resetting encoders.
      CommandScheduler.getInstance().schedule(m_robotContainer.zeroCommand());
      CommandScheduler.getInstance().schedule(m_robotContainer.unClampManipulator());

      
      Navx.setPitchZero(Navx.getPitch());
    }

    @Override
    public void teleopPeriodic () {
      if (Constants.useAprilTags && !suceededTeleReset) {
        if (m_robotContainer.resetOdometryWithAprilTag()) {
          suceededTeleReset = true;
        }
      }
    }

    @Override
    public void teleopExit () {
    }

    @Override
    public void testInit () {
      CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic () {
    }

    @Override
    public void testExit () {
    }
  }