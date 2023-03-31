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
  private final double pitchZero = -8.211; //should go in Constants
  private final double pitchDeadband = 5;
  private final double pitchVelocityDeadband = 0.5;


  private double direction;
  private int iterator;
  private boolean pitchVelocityCheck = false;
  private final double driveVelocity = 0.75;
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

    oddPitch = 0;

    
    pitch = (Navx.getPitch());
    
  }

  

 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    pitch = (Navx.getPitch());
 

    iterator++;

    if(iterator % 2 == 0){
      pitchVelocityCheck = (Math.abs(oddPitch - pitch ) <= pitchVelocityDeadband);
     }
    else{oddPitch = Navx.getPitch();}

    


    //trajectory to go forward 2 meters * sign
    /* Call should be:
    ConditionalCommand balance = new ConditionalCommand(Balance(m_chassis, 1), Balance(m_chassis, -1), Navx.getPitch() > 1).until(Math.abs(Navx.getPitch())) <= 5.0;
    */
    if(Navx.getPitch() < pitchZero){
      SwerveModuleState[] moduleStates = m_chassis.getKinematics().toSwerveModuleStates(new ChassisSpeeds(driveVelocity,0,0));
      m_chassis.setModuleStates(moduleStates);
    }
    else{
      SwerveModuleState[] moduleStates = m_chassis.getKinematics().toSwerveModuleStates(new ChassisSpeeds(-driveVelocity,0,0));
      m_chassis.setModuleStates(moduleStates);
    }



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_chassis.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((pitchVelocityCheck && Math.abs(Navx.getPitch()) <= Math.abs(pitchDeadband - pitchZero) )){
      System.out.println("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX " + Navx.getPitch());
      return true;}
return false;
  }

  public double getDirection() {
    return direction;
  }
}
