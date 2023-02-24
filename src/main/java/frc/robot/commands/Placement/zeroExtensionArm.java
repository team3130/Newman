// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Newman_Constants.Constants;
import frc.robot.subsystems.ExtensionArm;
import frc.robot.supportingClasses.ShuffleboardUpdated;

/** An example command that uses an example subsystem. */
public class zeroExtensionArm extends CommandBase implements ShuffleboardUpdated {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExtensionArm m_extensionArm;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public zeroExtensionArm(ExtensionArm subsystem) {
    m_extensionArm = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_extensionArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_extensionArm.spinExtensionArm(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<< HEAD:src/main/java/frc/robot/commands/MoveExtensionArm.java
    double y = m_xboxController.getRawAxis(0); // inverted?

    if (Math.abs(y) < Constants.kDeadband) {
      y = 0;
    }
    m_extensionArm.ExtendExtensionArm(y * extensionArmMaxSpeed); //that max is currently bs
    if (m_extensionArm.BrokeLimit()) {
      m_extensionArm.StopArm();
    }
=======
>>>>>>> main:src/main/java/frc/robot/commands/Placement/zeroExtensionArm.java
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_extensionArm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_extensionArm.hitLimitSwitch();
  }

  @Override
  public void updateValueFromShuffleboard() {

  }
}