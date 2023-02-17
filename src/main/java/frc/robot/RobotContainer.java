// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.FlipFieldOrriented;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.ZeroEverything;
import frc.robot.commands.ZeroWheels;
import frc.robot.sensors.Navx;
import frc.robot.subsystems.Chassis;
import frc.robot.Newman_Constants.Constants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  private static Joystick m_driverGamepad;
  private final Chassis m_chassis = new Chassis();


  public Chassis getChassis() {
    return m_chassis;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    m_driverGamepad = new Joystick(0);
    configureButtonBindings();

     m_chassis.setDefaultCommand(new TeleopDrive(m_chassis));
     m_chassis.resetOdometry(new Pose2d(new Translation2d(0,0),new Rotation2d(0)));
  }


  public static Joystick getDriverGamepad() {
    return m_driverGamepad;
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

    SmartDashboard.putData(new FlipFieldOrriented(m_chassis));
  }

}