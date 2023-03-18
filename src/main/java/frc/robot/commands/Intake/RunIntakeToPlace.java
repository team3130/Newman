// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.IntakeBeaterBar;
import frc.robot.subsystems.IntakePivot;

/** A command to run intake to the position we need to place game elements */
public class RunIntakeToPlace extends CommandBase {
  private final IntakeBeaterBar m_beaterBar; // the beater bar will need to be running to place
  private final Hopper m_hopper; // the hopper will need to be running to place
  private final IntakePivot m_pivot; // the arm will need to be pivoted to a certain place

  /**
   * Creates a new RunIntakeToPlace command
   *
   * @param beaterBar The IntakeBeaterBar subsystem
   * @param pivot the pivot subsystem
   * @param hopper the hopper subsystem
   */
  public RunIntakeToPlace(IntakeBeaterBar beaterBar, IntakePivot pivot, Hopper hopper) {
    m_beaterBar = beaterBar;
    m_pivot = pivot;
    m_hopper = hopper;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_beaterBar, m_pivot, m_hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  m_beaterBar.spin();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // spin the beater bar

    // if we are at the middle position then spin hopper
    //TODO: so if we aren't at the middle position hopper shouldn't run?
    if (m_pivot.atMiddlePos()) {
      m_hopper.spinHopper();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stop the intake and hopper motors
    m_beaterBar.stop();
    m_hopper.stopHopper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // never done, wait for button press
    return false;
  }
}
