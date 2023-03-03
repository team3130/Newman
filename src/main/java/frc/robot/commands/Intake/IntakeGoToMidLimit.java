// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeBeaterBar;
import frc.robot.subsystems.IntakePivot;

/** An example command that uses an example subsystem. */
public class IntakeGoToMidLimit extends CommandBase {
  private final IntakeBeaterBar m_beaterBar;
  private final IntakePivot m_pivot;

  /*
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeGoToMidLimit(IntakeBeaterBar subsystem1, IntakePivot subsystem2) {
    m_beaterBar = subsystem1;
    m_pivot = subsystem2;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_beaterBar, m_pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_pivot.lastLimitPosition == m_pivot.m_highPosition){
      m_pivot.movePivotMotor(-1);
    }
    if(m_pivot.lastLimitPosition == m_pivot.m_lowPosition){
      m_pivot.movePivotMotor(1);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_pivot.hitLimitSwitch(m_pivot.m_middlePosition)){m_pivot.stop();}
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
