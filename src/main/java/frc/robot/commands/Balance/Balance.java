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
  private double pitchZero =Constants.Balance.defaultPitchZero; //should go in Constants
  private final double pitchDeadband = Constants.Balance.pitchDeadband;
  private final double pitchVelocityDeadband = Constants.Balance.pitchVelocityDeadband;


  private double direction;
  private int iterator;
  private boolean pitchVelocityCheck = false;
  private final double driveVelocity = Constants.Balance.driveSpeed;
  private double oddPitch;
  private double pitch;


  private boolean nearZero; 
  private boolean drivingNegative;
  private boolean hasSwitched;
  private boolean timerIsOn;
  private boolean finished;
 
  private Timer timer = new Timer();
  private Timer safetyTimer = new Timer();

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
    pitchZero = Navx.getZeroPitch();

    oddPitch = 0;
    pitch = (Navx.getPitch());

    pitchVelocityCheck = false;
    drivingNegative = true;
    hasSwitched = false;
    iterator = 0;
    
    nearZero = false;
    timerIsOn = false;

    finished = false;

    timer.reset();
    timer.start();

    safetyTimer.reset();
    safetyTimer.start();
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
    nearZero = true;
    hasSwitched = true;
    timerIsOn = false;
  }

   if(!hasSwitched){
      timerIsOn = false;

      if(Navx.getPitch() < pitchZero){
    
        m_chassis.drive(driveVelocity, 0, 0, false);

      if(drivingNegative){ //maybe put a pitch  is around zero check for timer also 
        hasSwitched = true;
        drivingNegative = false;
      }
      else{hasSwitched = false;}
    }
      else{
        m_chassis.drive(-driveVelocity, 0, 0, false);

      if(!drivingNegative){
        hasSwitched = true;
        drivingNegative = true;
      }
      else{hasSwitched = false;}

      
    }

    


  }
    else{
      m_chassis.stopModules();

      if(!timerIsOn){
         timer.restart(); 
         timerIsOn = true;
      }
     
     if(nearZero){ 
      m_chassis.brakeModules();
          if(timer.hasElapsed(Constants.Balance.stablizationTime)){
            hasSwitched = false;
            finished = Math.abs(Navx.getPitch() - pitchZero) <= pitchDeadband;
       // timerIsOn = false;
      }
    }
      else {
        m_chassis.brakeModules();
          if(timer.hasElapsed(Constants.Balance.stablizationTime)){
            hasSwitched = false;
            // timerIsOn = false;
        }
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
    //return pitchVelocityDeadband && Math.abs(Navx.getPitch() - pitchZero) <= pitchDeadband;
    return finished || safetyTimer.hasElapsed(Constants.Balance.safetyTimeLimit);
   // return finished;
  }

  public double getDirection() {
    return direction;
  }
}
