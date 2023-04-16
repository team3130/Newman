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
  private boolean onRamp = false;
  private final double driveVelocity = Constants.Balance.driveSpeed;
  private final double pitchZero = -6.75;
  private final double pitchVelocityDeadband = 0.05;
  private SwerveModuleState[] moduleStates;
  private int iterator;
  private double oddPitch;
  private boolean pitchHasDropped;
  private double pitchCheckValue;

  private boolean distanceFlag;
  private boolean finished;

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
    distanceFlag = false;
    onRamp = false;
    state = State.TO_RAMP;
    iterator = 0;
    distanceToDrive = Constants.Balance.initSearchDistance;
    sign = -1;

    timer.reset();
    safetyTimer.reset();
    safetyTimer.start();

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
      iterator++;
      
      distanceToDrive = (distanceToDrive / iterator); 
      sign = (Navx.getPitch() < Navx.getZeroPitch()) ? 1 : -1;

      if(timer.hasElapsed(0.75)){
        timer.stop();
        finished = Math.abs(Navx.getPitch() - Navx.getZeroPitch()) <= Constants.Balance.pitchDeadband;
      }

    }
    else if (state == State.DRIVING){
      initPos = m_chassis.getPose2d().getTranslation().getX(); 

        if(!(Math.abs(Navx.getPitch() - Navx.getZeroPitch()) <= Constants.Balance.pitchDeadband) && Math.abs(m_chassis.getPose2d().getTranslation().getX() - initPos) <= distanceToDrive){
          m_chassis.drive(sign * driveVelocity,0,0, false);
        }
        else{
          m_chassis.stopModules();
          timer.restart();
          state = State.WAITING;
        }


    }
`




  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_chassis.stopModules();
    timer.stop();
    timer.reset();
    safetyTimer.stop();
    safetyTimer.reset();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
   // return finished || safetyTimer.hasElapsed(Constants.Balance.safetyTimeLimit); 
  }
}

