// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.IntakeBeaterBar;

/** An example command that uses an example subsystem. */
public class IntakeToggleBeaterBar extends CommandBase {
  private final IntakeBeaterBar m_beaterBar;
  private final Hopper m_hopper;
  /*
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeToggleBeaterBar(IntakeBeaterBar beaterbar, Hopper hopper) {
    m_beaterBar = beaterbar;
    m_hopper =hopper;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_beaterBar);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!m_beaterBar.isSpinning()){
      m_beaterBar.spin();
    }
    else{
      m_beaterBar.stop();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
