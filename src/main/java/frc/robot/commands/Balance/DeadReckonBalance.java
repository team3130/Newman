// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Balance;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.Newman_Constants.Constants;
import frc.robot.sensors.Navx;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ExampleSubsystem;

/** An example command that uses an example subsystem. */
public class DeadReckonBalance extends CommandBase {
  private final Chassis m_chassis;
  private boolean onRamp = false;
  private final double driveSpeed = 0.75;
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
  


  /*
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DeadReckonBalance(Chassis chassis) {
    m_chassis = chassis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    moduleStates = m_chassis.getKinematics().toSwerveModuleStates(new ChassisSpeeds(driveSpeed,0,0));
    m_chassis.setModuleStates(moduleStates);

    finished = false;
    distanceFlag = false;
    onRamp = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   if(!distanceFlag){
    if (Math.abs(Navx.getPitch() - pitchZero) >= Constants.Balance.changeForRampPitch && !onRamp) { //8 is how many degrees for the bot to understand it is on the ramp
      onRamp = true;
      
    }

    if(onRamp && Navx.getPitch() - pitchZero >= Constants.Balance.changeForRampPitch ){ //Assumes positive pitch is when the bot is tilted the second time
      //maybe instead of this condition for pitch read the direction change
      distanceFlag = true;
      m_chassis.stopModules();
      



    }



   }
   else{
    initPos = m_chassis.getPose2d().getTranslation().getX(); 

    if(Math.abs(m_chassis.getPose2d().getTranslation().getX() - initPos) <= Constants.Balance.tippedtoStationCenter ){ //0.4 is meters
      moduleStates = m_chassis.getKinematics().toSwerveModuleStates(new ChassisSpeeds(-driveSpeed,0,0));
      m_chassis.setModuleStates(moduleStates);
    }
    else{
      m_chassis.stopModules();
      finished = true;
    }

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
    return finished  ; //5 is how many degrees for the bot to know the ramp has tipped the other way
  }
}

