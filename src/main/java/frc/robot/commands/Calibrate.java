// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PlacementRotaryArm;
import frc.robot.subsystems.PlacementRotaryArm.Position;

/** A Calibrate command from the rotary arm */
public class Calibrate extends CommandBase {
  private final PlacementRotaryArm m_placementRotaryArm;

  private double speed = 0.4;

  private final Timer m_timer;

  private double timeOffset = 0.15;

  /**
   * Creates a new Calibrate command
   *
   * @param subsystem The subsystem used by this command.
   */
  public Calibrate(PlacementRotaryArm subsystem) {
    m_placementRotaryArm = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    m_timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_placementRotaryArm.runAtPercentOutput(speed);
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.hasElapsed(timeOffset)) {
      if (m_placementRotaryArm.goingUp()) {
        speed -= 0.01;
        m_timer.reset();
      }
      else if (m_placementRotaryArm.goingDown()) {
        speed += 0.01;
        m_timer.reset();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_placementRotaryArm.stop();
    System.out.println("FOUND VALUE: " + speed);
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(timeOffset * 1.5) && m_placementRotaryArm.isStationary();
  }
}
