// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Newman_Constants.Constants;

import frc.robot.Newman_Constants.Constants;
import frc.robot.commands.Placement.zeroExtensionArm;

public class Robot extends TimedRobot {
  private Timer timer;

  private RobotContainer m_robotContainer;
  boolean cameraIsGettingData = false;


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
    if (timer.hasElapsed(Constants.kResetTime)) {
      timer.reset();
      timer.stop();
      m_robotContainer.resetOdometry();
    }

    m_robotContainer.getLimelight().outputToShuffleBoard();
        // April tag odometry stuff
    if (cameraIsGettingData) {
     m_robotContainer.updatePosition();
    }
    else {
      if (m_robotContainer.tryUpdatePosition() > 5) {
        cameraIsGettingData = true;
      }
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().schedule(m_robotContainer.getAutonCmd());
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.zeroCommand();
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
