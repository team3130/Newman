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
  private final double pitchZero = -6.75; //should go in Constants
  private final double pitchDeadband = 3.5;
  private final double pitchVelocityDeadband = 0.1;


  private double direction;
  private int iterator;
  private int timerIterator;
  private boolean pitchVelocityCheck = false;
  private final double driveVelocity = 0.625;
  private double oddPitch;
  private double pitch;

  private boolean drivingPositive;
  private boolean hasSwitched;
 
  private Timer timer = new Timer();

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

    pitchVelocityCheck = false;
    drivingPositive = true;
    hasSwitched = false;
    iterator = 0;
    timerIterator = 0;
    

    timer.reset();
    timer.start();
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

    
  if( Math.abs(Navx.getPitch() - pitchZero) <= pitchDeadband){
    hasSwitched = true;
    timerIterator = 0;
  }

   if(!hasSwitched){
    timerIterator = 0;

    if(Navx.getPitch() < pitchZero){
      SwerveModuleState[] moduleStates = m_chassis.getKinematics().toSwerveModuleStates(new ChassisSpeeds(driveVelocity,0,0));
      m_chassis.setModuleStates(moduleStates);
      if(drivingPositive){ //maybe put a pitch  is around zero check for timer also 
        hasSwitched = true;
        drivingPositive = false;
      }
      else{hasSwitched = false;}
    }
    else{
      SwerveModuleState[] moduleStates = m_chassis.getKinematics().toSwerveModuleStates(new ChassisSpeeds(-driveVelocity,0,0));
      m_chassis.setModuleStates(moduleStates);
      if(!drivingPositive){
        hasSwitched = true;
        drivingPositive = true;
      }
      else{hasSwitched = false;}

      
    }

    


  }
    else{
      m_chassis.stopModules();

      if(timerIterator == 0){
         timer.restart(); 
         timerIterator++;
      }
      
      if(timer.hasElapsed(1.0)){
        hasSwitched = false;
      }
  
      

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    m_chassis.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pitchVelocityCheck && Math.abs(Navx.getPitch() - pitchZero) <= pitchDeadband;
  }

  public double getDirection() {
    return direction;
  }
}
