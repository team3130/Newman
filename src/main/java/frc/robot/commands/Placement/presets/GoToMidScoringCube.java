// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement.presets;

import frc.robot.Newman_Constants.Constants;
import frc.robot.subsystems.ExtensionArm;
import frc.robot.subsystems.RotaryArm;


/**
 * A preset command to go to the high position for the rotary arm.
 * Inherits {@link PlacementSetpoint} to handle the logic for where it should go.
 */
public class GoToMidScoringCube extends PlacementSetpoint {

  /**
   * Makes a new command to go to the high preset for rotary and extension arms.
   * @param rotaryArm the singleton for rotary arm.
   * @param extensionArm the singleton for Extension arm.
   */
  public GoToMidScoringCube(RotaryArm rotaryArm, ExtensionArm extensionArm) {
    // extension arm, rotary arm, the rotary arm high position, the max extension of the extension arm, the angle before bumpers.
    super(rotaryArm, extensionArm, Constants.midPosition);
  }
}
