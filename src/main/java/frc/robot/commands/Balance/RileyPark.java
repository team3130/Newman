// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Balance;

import frc.robot.sensors.Navx;
import frc.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class RileyPark extends CommandBase {
  private final Chassis m_chassis;

  /*
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RileyPark(Chassis chassis, double direction) {
    m_chassis = chassis;
    this.direction = direction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  private final double direction;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = Navx.getAngle();
    
    m_chassis.turnToAngle(angle + direction + Math.PI / 2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
