// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Newman_Constants.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.PlacementExtensionArm;
import frc.robot.subsystems.PlacementRotaryArm;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  private static Joystick m_driverGamepad;
  private static Joystick m_weaponsGamepad;
  private final Chassis m_chassis = new Chassis();
  private final PlacementExtensionArm m_placementExtensionArm = new PlacementExtensionArm();
  private final PlacementRotaryArm m_placementRotaryArm = new PlacementRotaryArm();


  public Chassis getChassis() {
    return m_chassis;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    m_driverGamepad = new Joystick(0);
    m_weaponsGamepad = new Joystick(1);
    configureButtonBindings();

     m_chassis.setDefaultCommand(new TeleopDrive(m_chassis));
  }

  public static Joystick getDriverGamepad() {
    return m_driverGamepad;
  }

  public static Joystick getWeaponsGamepad() {
    return m_weaponsGamepad;
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverGamepad, Constants.Buttons.LST_BTN_A).whileTrue(new ZeroWheels(m_chassis));
    new JoystickButton(m_driverGamepad, Constants.Buttons.LST_BTN_B).whileTrue(new ZeroEverything(m_chassis));
    new JoystickButton(m_driverGamepad, Constants.Buttons.LST_BTN_X).whileTrue(new Calibrate(m_placementRotaryArm));
    new JoystickButton(m_weaponsGamepad, Constants.Buttons.LST_AXS_LTRIGGER).whileTrue(new MidPlacement(m_placementRotaryArm));
    new JoystickButton(m_weaponsGamepad, Constants.Buttons.LST_BTN_LBUMPER).whileTrue(new HighPlacement(m_placementRotaryArm));
    new JoystickButton(m_weaponsGamepad, Constants.Buttons.LST_AXS_RTRIGGER).whileTrue(new ExtendPlacement(m_placementExtensionArm));
    new JoystickButton(m_weaponsGamepad, Constants.Buttons.LST_BTN_RBUMPER).whileTrue(new CollapsePlacement(m_placementExtensionArm));
    new JoystickButton(m_weaponsGamepad, Constants.Buttons.LST_BTN_Y).whileTrue(new ZeroPlacement(m_placementExtensionArm));
    
    SmartDashboard.putData(new FlipFieldOrriented(m_chassis));
  }

}