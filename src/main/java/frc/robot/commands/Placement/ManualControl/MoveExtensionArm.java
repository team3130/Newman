// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement.ManualControl;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Newman_Constants.Constants;
<<<<<<< HEAD:src/main/java/frc/robot/commands/Placement/MoveExtensionArm.java
import frc.robot.subsystems.RotaryArm;
import frc.robot.supportingClasses.ShuffleboardUpdated;
=======
import frc.robot.subsystems.ExtensionArm;
>>>>>>> main:src/main/java/frc/robot/commands/Placement/ManualControl/MoveExtensionArm.java

/** A command to move the extension arm based off the joysticks */
public class MoveExtensionArm extends CommandBase {
  private final ExtensionArm m_extensionArm;
  private final RotaryArm m_rotaryArm;

  public Joystick m_xboxController;

  /*public boolean justHitLimit = false;*/

  /**
   * Creates a new Move Extension Arm command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveExtensionArm(ExtensionArm subsystem, RotaryArm rsubsystem, Joystick m_xboxController) {
    m_rotaryArm = rsubsystem;
    m_extensionArm = subsystem;
    this.m_xboxController = m_xboxController;
//    boolean flag = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_extensionArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double y = -m_xboxController.getRawAxis(Constants.Buttons.LST_AXS_LJOYSTICKY); // inverted
    y = y * Math.abs(y);

<<<<<<< HEAD:src/main/java/frc/robot/commands/Placement/MoveExtensionArm.java
    if (Math.abs(y) < Constants.kDeadband || (y < 0 && m_extensionArm.hitLimitSwitch())) {
      y = 0;
    }
    if(m_extensionArm.getDistanceExtensionArm()>)
=======
    m_extensionArm.spinExtensionArm(y); //that max is currently bs
>>>>>>> main:src/main/java/frc/robot/commands/Placement/ManualControl/MoveExtensionArm.java
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_extensionArm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
