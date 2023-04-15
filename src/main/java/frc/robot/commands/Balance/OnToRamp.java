package frc.robot.commands.Balance;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import frc.robot.Newman_Constants.Constants;
import frc.robot.sensors.Navx;
import frc.robot.subsystems.Chassis;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class OnToRamp extends CommandBase {
  private final Chassis m_chassis;
  private Timer timer = new Timer();
  private final double driveVelocity = Constants.Balance.driveSpeed;
  private int direction;

  private double initHeading = Navx.getYaw();
 

  /*
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public OnToRamp(Chassis chassis, boolean drivePositive) {
    m_chassis = chassis;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);

    direction = drivePositive ? 1 : -1;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    initHeading = Navx.getYaw();

 
  }

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angVelocity = Math.toRadians( initHeading - Navx.getYaw());

    SwerveModuleState[] moduleStates = m_chassis.getKinematics().toSwerveModuleStates(new ChassisSpeeds(driveVelocity * direction,0,Constants.Balance.HeadingkP * angVelocity));
    m_chassis.setModuleStates(moduleStates);
   
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
    return Math.abs(Navx.getPitch() - Navx.getZeroPitch()) >= Constants.Balance.changeForRampPitch || timer.hasElapsed(Constants.Balance.safetyTimeLimit);
  }
}
