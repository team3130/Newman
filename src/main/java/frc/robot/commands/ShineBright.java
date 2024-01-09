// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDs;

/** An example command that uses an example subsystem. */
public class ShineBright extends CommandBase {

  /**
   * The subsystem that this command requires
   */
  private final LEDs m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShineBright(LEDs subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  /**
   * Called when the command is initially scheduled.
   */
  @Override
  public void initialize() {

  }

  /**
   * Called every time the scheduler runs while the command is scheduled.
   */
  @Override
  public void execute() {
      for (var i = 0; i < LEDs.m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        LEDs.m_ledBuffer.setRGB(i, 255, 0, 0);
      }

      LEDs.m_led.setData(LEDs.m_ledBuffer);
  }

  /**
   * Called once the command ends or is interrupted.
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {}

  /**
   * Returns true when the command should end.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
