// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Newman_Constants.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis;

/** An example command that uses an example subsystem. */
public class TeleopDrive extends CommandBase {
  private final Chassis m_chassis;

  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  private Joystick m_xboxController;

  /**
   * Creates a new ExampleCommand.
   *
   * @param chassis The subsystem used by this command.
   */
  public TeleopDrive(Chassis chassis) {
    m_chassis = chassis;
    // Use addRequirements() here to declare subsystem dependencies.
    m_requirements.add(chassis);
    xLimiter = new SlewRateLimiter(Constants.kMaxAccelerationDrive);
    yLimiter = new SlewRateLimiter(Constants.kMaxAccelerationDrive);
    turningLimiter = new SlewRateLimiter(Constants.kMaxAccelerationAngularDrive);
    m_xboxController = RobotContainer.getDriverGamepad();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double y = -m_xboxController.getRawAxis(0); // left stick y-axis (y-axis is inverted)
    double x = -m_xboxController.getRawAxis(1); // left stick x-axis
    double theta = -m_xboxController.getRawAxis(4); // right stick x-axis

    // apply dead-band
    if (Math.abs(x) < Constants.kDeadband && Math.abs(y) < Constants.kDeadband) {
      x = 0;
      y = 0;
    }
    theta = Math.abs(theta) > Constants.kDeadband ? theta : 0.0;

    x = xLimiter.calculate(x * Constants.kPhysicalMaxSpeedMetersPerSecond);
    y = yLimiter.calculate(y * Constants.kPhysicalMaxSpeedMetersPerSecond);
    theta = turningLimiter.calculate(theta) * Constants.kPhysicalMaxSpeedMetersPerSecond;


  SwerveModuleState[] moduleStates;
    if (m_chassis.getFieldRelative()) {
      moduleStates = m_chassis.getKinematics().toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, theta, m_chassis.getRotation2d()));
    }
    else {
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
