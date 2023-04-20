// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Balance;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.Newman_Constants.Constants;
import frc.robot.sensors.Navx;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ExampleSubsystem;

/** An example command that uses an example subsystem. */
public class SearchBalance extends CommandBase {
  private final Chassis m_chassis;
  private final double driveVelocity = Constants.Balance.driveSpeed;
  private SwerveModuleState[] moduleStates;
  private int iterator = 1;
  

  
  private boolean finished;

  private boolean useAprilTags;
  
  private double initPos;

  private enum State{
    TO_RAMP,
    WAITING,
    DRIVING
  }
  private State state;
  private Timer timer = new Timer();
  private Timer safetyTimer = new Timer();
  private double distanceToDrive;
  private int sign;
  private double initRotation; 

  public SearchBalance(Chassis chassis) {
    m_chassis = chassis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_chassis.stopModules();

    finished = false;
    
    
    state = State.TO_RAMP;
    iterator = 1;
    distanceToDrive = Constants.Balance.initSearchDistance;
    sign = -1;

    timer.reset();
    safetyTimer.start();
    safetyTimer.reset();

    useAprilTags = m_chassis.getAprilTags();
    m_chassis.setAprilTagUsage(false);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(state == State.TO_RAMP){
      m_chassis.drive(-driveVelocity, 0, 0, false);

      if(Math.abs(Navx.getPitch() - Navx.getZeroPitch()) >= Constants.Balance.changeForRampPitch){
        m_chassis.stopModules();
        timer.restart();
        state = State.WAITING;
      }

    }
    else if (state == State.WAITING){
      m_chassis.brakeModules();

      sign = (Navx.getPitch() < Navx.getZeroPitch()) ? 1 : -1;

      if(timer.hasElapsed(Constants.Balance.stablizationTime)){
        timer.reset();
        timer.stop();
        finished = Math.abs(Navx.getPitch() - Navx.getZeroPitch()) <= Constants.Balance.pitchDeadband;
        initPos = m_chassis.getX();
         
        state = State.DRIVING;
      }

    }
    else if (state == State.DRIVING){
      

        if((Math.abs(Navx.getPitch() - Navx.getZeroPitch()) <= Constants.Balance.pitchDeadband) || Math.abs(m_chassis.getX() - initPos) >= distanceToDrive){
          m_chassis.stopModules();
          timer.reset();
          timer.stop();
          iterator++;
          distanceToDrive = distanceToDrive / 2;
          state = State.WAITING;
        }
         
        else{
          m_chassis.drive(sign * driveVelocity,0,0, false);
          
        }


    }





  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_chassis.stopModules();
    timer.reset();
    timer.stop();
    safetyTimer.reset();
    safetyTimer.stop();
    m_chassis.setAprilTagUsage(Constants.useAprilTags);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return finished;
   return finished || safetyTimer.hasElapsed(Constants.Balance.safetyTimeLimit); 
  }
}

