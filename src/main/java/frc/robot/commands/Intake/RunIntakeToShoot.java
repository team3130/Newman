// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.IntakeBeaterBar;
import frc.robot.subsystems.IntakePivot;

/** An example command that uses an example subsystem. */
public class RunIntakeToShoot extends CommandBase {
  private final IntakeBeaterBar m_beaterBar;
  private final Hopper m_hopper;
  private final IntakePivot m_pivot;
  /*
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunIntakeToShoot(IntakeBeaterBar beaterbar, IntakePivot pivot, Hopper hopper) {
    m_beaterBar = beaterbar;
    m_hopper = hopper;
    m_pivot = pivot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_beaterBar, m_hopper, m_pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_beaterBar.spin();
    if (m_pivot.atMiddlePos() && !m_hopper.hasNards()){ //if pivot is at middle and the hopper is empty run hopper
      m_hopper.spinHopper(); //may need gentleSpin
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_beaterBar.stop();
    m_hopper.stopHopper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_hopper.hasNards();
  } //once there is something in hopper intake should stop
}
