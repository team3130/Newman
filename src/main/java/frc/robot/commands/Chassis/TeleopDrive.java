// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Newman_Constants.Constants;
import frc.robot.subsystems.Chassis;

/** A command to drive in teleop */
public class TeleopDrive extends CommandBase {
  private final Chassis m_chassis; // chassis subsystem

  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter; // acceleration limiters

  private final Joystick m_xboxController; // the controller we use to drive

  /**
   * Creates a new TeleopDrive command
   * Initializes slew rate limiters to limit acceleration
   *
   * @param chassis The subsystem used by this command.
   * @param xboxController the controller that we use to drive
   */
  public TeleopDrive(Chassis chassis, Joystick xboxController) {
    m_chassis = chassis;
    // Use addRequirements() here to declare subsystem dependencies.
    m_requirements.add(chassis);
    xLimiter = new SlewRateLimiter(Constants.kMaxAccelerationDrive);
    yLimiter = new SlewRateLimiter(Constants.kMaxAccelerationDrive);
    turningLimiter = new SlewRateLimiter(Constants.kMaxAccelerationAngularDrive);
    m_xboxController = xboxController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double y = -m_xboxController.getRawAxis(Constants.Buttons.LST_AXS_LJOYSTICKX); // left stick y-axis (y-axis is inverted)
    double x = -m_xboxController.getRawAxis(Constants.Buttons.LST_AXS_LJOYSTICKY); // left stick x-axis
    double theta = -m_xboxController.getRawAxis(Constants.Buttons.LST_AXS_RJOYSTICKX); // right stick x-axis

    // square the inputs
    y = y * Math.abs(y);
    x = x * Math.abs(x);

    if (m_chassis.isTurtleMode()){ //cube inputs
      x = x * Math.abs(x);
      y = y * Math.abs(y);
    }


    // apply dead-band
    if (Math.abs(x) < Constants.kDeadband) {
      x = 0;
    }
    if (Math.abs(y) < Constants.kDeadband) {
      y = 0;
    }
    theta = Math.abs(theta) > Constants.kDeadband ? theta : 0.0;

    // apply slew rate limiter which also converts to m/s and rad.s
    x = xLimiter.calculate(x * Constants.kPhysicalMaxSpeedMetersPerSecond);
    y = yLimiter.calculate(y * Constants.kPhysicalMaxSpeedMetersPerSecond);
    theta = turningLimiter.calculate(theta) * Constants.kPhysicalMaxSpeedMetersPerSecond;

    // holder for the module states
    SwerveModuleState[] moduleStates;

    if (m_chassis.getFieldRelative()) {
      // field relative states
      moduleStates = m_chassis.getKinematics().toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, theta, m_chassis.getRotation2d()));
    }
    else {
      // robot oriented states
      moduleStates = m_chassis.getKinematics().toSwerveModuleStates(new ChassisSpeeds(x,y,theta));
    }
    m_chassis.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_chassis.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
