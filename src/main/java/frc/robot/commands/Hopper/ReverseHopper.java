// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.IntakeBeaterBar;
import frc.robot.subsystems.IntakePivot;

/** An example command that uses an example subsystem. */
public class ReverseHopper extends CommandBase {
  private final Hopper m_hopper;
  private final IntakeBeaterBar m_beater;
  private final IntakePivot m_pivot;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ReverseHopper(Hopper hopper, IntakeBeaterBar beater, IntakePivot pivot) {
    m_hopper = hopper;
    m_beater = beater;
    m_pivot = pivot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hopper, beater, pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_hopper.spitToDumpHopper();
    if (m_pivot.atMiddlePos()){
      m_beater.reverse();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hopper.stopHopper();
    m_beater.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
