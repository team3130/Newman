// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Newman_Constants.Constants;

public class Robot extends TimedRobot {
  private Timer timer;
  private RobotContainer m_robotContainer;

  private boolean haveResetManually = false;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    timer = new Timer();
    timer.reset();
    timer.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.periodic();
    if (timer.hasElapsed(Constants.kResetTime)) {
      if (!haveResetManually) {
        m_robotContainer.resetOdometryWithoutApril();
        haveResetManually = true;
      }
      else {
        if (m_robotContainer.resetOdometryWithAprilTag()) {
          timer.reset();
          timer.stop();
        }
      }
    }
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

    @Override
    public void autonomousInit () {
      CommandScheduler.getInstance().cancelAll();
      CommandScheduler.getInstance().schedule(m_robotContainer.getAutonCmd());
    
    }

    @Override
    public void autonomousPeriodic () {
    }

    @Override
    public void autonomousExit () {
    }

    @Override
    public void teleopInit () {
      CommandScheduler.getInstance().cancelAll();
      CommandScheduler.getInstance().schedule(m_robotContainer.zeroCommand());
    }

    @Override
    public void teleopPeriodic () {

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