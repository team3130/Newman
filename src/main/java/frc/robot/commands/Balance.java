// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Newman_Constants.Constants;
import frc.robot.sensors.Navx;
import frc.robot.subsystems.Chassis;
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
   * @param subsystem The subsystem used by this command.
   */
  public Balance(Chassis chassis) {
    m_chassis = chassis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }
  private double angle;
  private double pitch;
  private double roll;
  private double tilt;
  private double accelerationPitch;
  private double accelerationRoll;
  private double accelerationTilt;
  private double tiltSum = 0;
  

  private static ShuffleboardTab tab = Shuffleboard.getTab("Chassis");

  private final GenericEntry P = tab.add("Balancer P", Constants.BalanceKp).getEntry();
  private final GenericEntry I = tab.add("Balancer I", Constants.BalanceKi).getEntry();
  private final GenericEntry D = tab.add("Balancer D", Constants.BalanceKd).getEntry();
  private final GenericEntry F = tab.add("Balancer F", Constants.BalanceKf).getEntry();
  private final GenericEntry Constrain = tab.add("Balancer F", Constants.BalanceConstrain).getEntry();


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle = Navx.getAngle();
    accelerationPitch = Math.toRadians(Navx.getPitchAccel());
    accelerationRoll = Math.toRadians(Navx.getRollAccel());
    pitch = Math.toRadians(Navx.getPitch());
    roll = Math.toRadians(Navx.getRoll());
    
    tilt = Math.atan(Math.sqrt(Math.pow(Math.tan(pitch),2) + Math.pow(Math.tan(roll),2)));
    accelerationTilt = Math.atan(Math.sqrt(Math.pow(Math.tan(accelerationPitch),2) + Math.pow(Math.tan(accelerationRoll),2)));
    tiltSum += tilt;
    if (tiltSum < -Constrain.getDouble(Constants.BalanceConstrain)) {
      tiltSum = -Constrain.getDouble(Constants.BalanceConstrain);
    } 
    else if (tiltSum > Constrain.getDouble(Constants.BalanceConstrain)) {
      tiltSum = Constrain.getDouble(Constants.BalanceConstrain);
    } 
     
    int sign = 0;
    if (pitch > roll) {
      if (pitch > 0) 
        sign = 1;
      else 
        sign = -1;
    }
    else {
      if (roll > 0)
        sign = 1;
      else 
        sign = -1;
    }

    //TODO: Tune PID values
    double magnitude = (P.getDouble(Constants.BalanceKp) * tilt + I.getDouble(Constants.BalanceKi) * tiltSum + D.getDouble(Constants.BalanceKd) * accelerationTilt + F.getDouble(Constants.BalanceKf)) * sign;
    double direction = Math.atan2(Math.tan(roll),Math.tan(pitch)); //in radians???
    
    double x = magnitude * Math.cos(direction);
    double y = magnitude * Math.sin(direction);

    SwerveModuleState[] moduleStates;
    if (m_chassis.getFieldRelative()) {
      moduleStates = m_chassis.getKinematics().toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, 0, m_chassis.getRotation2d()));
    }
    else {
      moduleStates = m_chassis.getKinematics().toSwerveModuleStates(new ChassisSpeeds(x,y,0));
    }
    m_chassis.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    angle = Navx.getAngle();

    double direction = Math.atan2(Math.tan(roll),Math.tan(pitch)); //in radians???
    
    m_chassis.turnToAngle(angle + direction + Math.PI / 2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
