// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Balance;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.sensors.Navx;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ExampleSubsystem;

/** An example command that uses an example subsystem. */
public class DeadReckonBalance extends CommandBase {
  private final Chassis m_chassis;
  private boolean onRamp = false;
  private final double driveSpeed = 0.75;
  private final double pitchZero = -8.221;
  private final double pitchVelocityDeadband = 0.05;
  private SwerveModuleState[] moduleStates;
  private int iterator;
  private double oddPitch;
  private boolean pitchHasDropped;
  private double pitchCheckValue;

  

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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Math.abs(Navx.getPitch() - pitchZero) >= 10.0 && !onRamp) { //10.0 is how many degrees for the bot to understand it is on the ramp
      onRamp = true;
      pitchCheckValue = Navx.getPitch();
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
    return onRamp && Math.abs(Navx.getPitch() - pitchCheckValue) >= 5.0  ; //5 is how many degrees for the bot to know the ramp has tipped the other way
  }
}

