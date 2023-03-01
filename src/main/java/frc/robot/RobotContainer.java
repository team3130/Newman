// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.FlipFieldOrriented;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.ZeroEverything;
import frc.robot.commands.ZeroWheels;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ExtensionArm;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.RotaryArm;
import frc.robot.Newman_Constants.Constants;
import frc.robot.subsystems.Hopper;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  private static Joystick m_driverGamepad;
  private static Joystick m_weaponsGamepad;
  private final Chassis m_chassis;
  private final ExtensionArm m_extensionArm;
  private final RotaryArm m_rotaryArm;
  private final Hopper m_hopper;
  private final Manipulator m_handGrabber;

  public Chassis getChassis() {
    return m_chassis;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    m_driverGamepad = new Joystick(0);
    m_weaponsGamepad = new Joystick(1);

    m_chassis = new Chassis();

    Mechanism2d arm = new Mechanism2d(4, 2);
    MechanismRoot2d root = arm.getRoot("arm", 5, 5);
    MechanismLigament2d zero = new MechanismLigament2d("retracted", Constants.kExtensionArmLengthExtended / 2, -90);
    root.append(zero);

    SmartDashboard.putData("Arm", arm);

    m_extensionArm =  new ExtensionArm(zero, m_weaponsGamepad);
    m_rotaryArm = new RotaryArm(zero);

    m_handGrabber = new Manipulator();
    m_hopper = new Hopper();

    m_chassis.setDefaultCommand(new TeleopDrive(m_chassis));

    // idk if this is right
    m_rotaryArm.setDefaultCommand(new MoveRotaryArm(m_rotaryArm));
    m_extensionArm.setDefaultCommand(new MoveExtensionArm(m_extensionArm, m_rotaryArm));

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
    new JoystickButton(m_weaponsGamepad, Constants.Buttons.LST_BTN_X).whileTrue(new MoveExtensionArm(m_extensionArm, m_rotaryArm));

    SmartDashboard.putData(new FlipFieldOrriented(m_chassis));
  }

  public void resetOdometry() {
    m_chassis.resetOdometry(new Pose2d(0, 0, new Rotation2d()));
  }

}