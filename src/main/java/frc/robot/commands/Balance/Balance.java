// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Balance;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Newman_Constants.Constants;
import frc.robot.commands.Chassis.ZeroEverything;
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
  private double sign;


  /**
   * Creates a new ExampleCommand.
   *
   * @param chassis The subsystem used by this command.
   */
  public Balance(Chassis chassis, int sign) {
    m_chassis = chassis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_chassis);
    this.sign = sign;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    fVelocityTilt = new MedianFilter(5);
    AccelerationTilt = 0;
    prev = 0;


    driveVelocity = 0.5; //Arbitrary Speed - Tune later
    initPitch = Math.toRadians(Navx.getPitch());
    pitch = Math.toRadians(Navx.getPitch());
    roll = Math.toRadians(Navx.getRoll());
    direction = Math.atan2(Math.tan(pitch),Math.tan(roll));
  }

  private double direction;

  private double driveVelocity;
  private double initPitch;
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

  /*private final GenericEntry P = tab.add("Balancer P", Constants.BalanceKp).getEntry();
  private final GenericEntry D = tab.add("Balancer D", Constants.BalanceKd).getEntry();
  private final GenericEntry F = tab.add("Balancer F", Constants.BalanceKf).getEntry();
  private final GenericEntry getDirection = tab.add("Direction", 0).getEntry();
  private final GenericEntry getTilt = tab.add("Tilt", 0).getEntry();
  private final GenericEntry getTiltVelocity = tab.add("Tilt Velocity", 0).getEntry(); */

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    velocityPitch = Math.toRadians(Navx.getPitchVelocity());
    velocityRoll = Math.toRadians(Navx.getRollVelocity());
    pitch = Math.toRadians(Navx.getPitch());
    roll = Math.toRadians(Navx.getRoll());


    //trajectory to go forward 2 meters * sign
    /* Call should be:
    ConditionalCommand balance = new ConditionalCommand(Balance(m_chassis, 1), Balance(m_chassis, -1), Navx.getPitch() > 1).until(Math.abs(Navx.getPitch())) <= 5.0;
    */
    SwerveModuleState[] moduleStates = m_chassis.getKinematics().toSwerveModuleStates(new ChassisSpeeds(driveVelocity * sign,0,0));

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

    return false;
  }

  public double getDirection() {
    return direction;
  }
}
