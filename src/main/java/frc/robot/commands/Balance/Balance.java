// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Balance;

import frc.robot.Newman_Constants.Constants;
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

  /**
   * Creates a new ExampleCommand.
   *
   * @param chassis The subsystem used by this command.
   */
  public Balance(Chassis chassis) {
    m_chassis = chassis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    fVelocityTilt = new MedianFilter(5);
    AccelerationTilt = 0;
    prev = 0;

    pitch = Math.toRadians(Navx.getPitch());
    roll = Math.toRadians(Navx.getRoll());
    direction = Math.atan2(Math.tan(pitch),Math.tan(roll));
  }

  private double direction;
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

  private final GenericEntry P = tab.add("Balancer P", Constants.BalanceKp).getEntry();
  private final GenericEntry D = tab.add("Balancer D", Constants.BalanceKd).getEntry();
  private final GenericEntry F = tab.add("Balancer F", Constants.BalanceKf).getEntry();
  private final GenericEntry getDirection = tab.add("Direction", 0).getEntry();
  private final GenericEntry getTilt = tab.add("Tilt", 0).getEntry();
  private final GenericEntry getTiltVelocity = tab.add("Tilt Velocity", 0).getEntry();

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    velocityPitch = Math.toRadians(Navx.getPitchVelocity());
    velocityRoll = Math.toRadians(Navx.getRollVelocity());
    pitch = Math.toRadians(Navx.getPitch());
    roll = Math.toRadians(Navx.getRoll());

    tilt = Math.atan(Math.sqrt(Math.pow(Math.tan(pitch),2) + Math.pow(Math.tan(roll),2)));
    VelocityTilt = Math.atan(Math.sqrt(Math.pow(Math.tan(velocityPitch),2) + Math.pow(Math.tan(velocityRoll),2)));

    VelocityTilt = fVelocityTilt.calculate(VelocityTilt);

    if (prev != 0) 
      AccelerationTilt = VelocityTilt - prev;
    prev = VelocityTilt;

    getTilt.setDouble(tilt);
    getTiltVelocity.setDouble(VelocityTilt);

    double magnitude = 
      P.getDouble(Constants.BalanceKp) * VelocityTilt + 
      D.getDouble(Constants.BalanceKd) * AccelerationTilt + 
      F.getDouble(Constants.BalanceKf);
    double direction = Math.atan2(Math.tan(pitch),Math.tan(roll)); 
    getDirection.setDouble(direction * 180 / Math.PI);

    double x = magnitude * Math.cos(direction);
    double y = magnitude * Math.sin(direction);

    SwerveModuleState[] moduleStates = m_chassis.getKinematics().toSwerveModuleStates(new ChassisSpeeds(x,y,0));
    m_chassis.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SwerveModuleState[] moduleStates = m_chassis.getKinematics().toSwerveModuleStates(new ChassisSpeeds(0,0,0));
    m_chassis.setModuleStates(moduleStates);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (tilt < 0.1 && VelocityTilt < 0.1) {
      return true;
    }
    return false;
  }

  public double getDirection() {
    return direction;
  }
}
