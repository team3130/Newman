// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Newman_Constants.Constants;
import frc.robot.commands.Chassis.FlipFieldOriented;
import frc.robot.commands.Chassis.GoToClosestPlaceToPlace;
import frc.robot.commands.Chassis.TeleopDrive;
import frc.robot.commands.Chassis.ZeroEverything;
import frc.robot.commands.Hopper.ReverseHopper;
import frc.robot.commands.Hopper.SpinHopper;
import frc.robot.commands.Hopper.UnjamHopper;
import frc.robot.commands.Intake.ToggleIntake;
import frc.robot.commands.Manipulator.ToggleGrabber;
import frc.robot.commands.Placement.*;
import frc.robot.controls.JoystickTrigger;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.*;
import frc.robot.supportingClasses.AutonManager;
import frc.robot.supportingClasses.OdoPosition;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /**
   * Auton manager is the object that handles the loading of auton paths
   */
  protected AutonManager m_autonManager;
  private static Joystick m_driverGamepad;
  private static Joystick m_weaponsGamepad;
  private final Chassis m_chassis;
  private final PlacementExtensionArm m_placementExtensionArm = new PlacementExtensionArm();
  private final PlacementRotaryArm m_placementRotaryArm = new PlacementRotaryArm();
  private final Manipulator m_manipulator = new Manipulator();

  private final Hopper m_hopper;
  private final IntakePivot m_pivot;
  private final Limelight m_limelight;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    m_driverGamepad = new Joystick(0);
    m_weaponsGamepad = new Joystick(1);

    m_limelight = new Limelight();

    m_chassis = new Chassis(m_limelight);
    m_hopper = new Hopper();
    m_pivot = new IntakePivot();

    m_autonManager = new AutonManager(m_chassis);

    m_chassis.setDefaultCommand(new TeleopDrive(m_chassis, m_driverGamepad));

    m_placementRotaryArm.setDefaultCommand(new MoveRotaryArm(m_placementRotaryArm, m_weaponsGamepad));
    m_placementExtensionArm.setDefaultCommand(new MoveExtensionArm(m_placementExtensionArm, m_weaponsGamepad));


    configureButtonBindings();
    vomitShuffleBoardData();
  }

  /**
   * adds the subsystem {@link edu.wpi.first.util.sendable.Sendable} objects to a 'Subsystems' shuffleboard tab
   */
  public void vomitShuffleBoardData() {
    if (Constants.debugMode) {
      ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
      tab.add(m_chassis);
      tab.add(m_placementExtensionArm);
      tab.add(m_placementRotaryArm);
      tab.add(m_manipulator);
      tab.add(m_hopper);
      tab.add(m_pivot);
      //m_chassis.shuffleboardVom(Shuffleboard.getTab("Swerve Modules"));
    }
  }

  /**
   * This shouldn't be necessary as we can just pass the initialized object,
   * However this can be here just in case we need it last minute
   *
   * @return the driver gamepad
   */
  public static Joystick getDriverGamepad() {
    return m_driverGamepad;
  }

  /**
   * This shouldn't be necessary as we can just pass the initialized object,
   * However this can be here just in case we need it last minute
   *
   * @return the weapons game pad
   */
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
    new JoystickButton(m_driverGamepad, Constants.Buttons.LST_BTN_RBUMPER).onTrue(new ToggleIntake(m_pivot));

    new JoystickButton(m_driverGamepad, Constants.Buttons.LST_BTN_A).whileTrue(new FlipFieldOriented(m_chassis));
    new JoystickButton(m_driverGamepad, Constants.Buttons.LST_BTN_B).whileTrue(new ZeroEverything(m_chassis));
    new JoystickButton(m_driverGamepad, Constants.Buttons.LST_BTN_Y).whileTrue(new GoToClosestPlaceToPlace(m_chassis, m_autonManager));
    
    new JoystickButton(m_weaponsGamepad, Constants.Buttons.LST_BTN_A).whileTrue(new SpinHopper(m_hopper));
    new JoystickButton(m_weaponsGamepad, Constants.Buttons.LST_BTN_Y).whileTrue(new ReverseHopper(m_hopper, m_pivot));
    new JoystickButton(m_weaponsGamepad, Constants.Buttons.LST_BTN_X).whileTrue(new UnjamHopper(m_hopper));
    new JoystickButton(m_weaponsGamepad, Constants.Buttons.LST_BTN_RBUMPER).onTrue(new ToggleBrake(m_placementRotaryArm));

    new JoystickTrigger(m_weaponsGamepad, Constants.Buttons.LST_AXS_LTRIGGER).onTrue(new ToggleGrabber(m_manipulator));

    new JoystickButton(m_weaponsGamepad, Constants.Buttons.LST_BTN_LBUMPER).whileTrue(new HighRotary(m_placementRotaryArm, m_placementExtensionArm));
/*
    new JoystickTrigger(m_weaponsGamepad, Constants.Buttons.LST_AXS_LTRIGGER).whileTrue(new MidRotary(m_placementRotaryArm, m_placementExtensionArm));
*/
    new JoystickTrigger(m_weaponsGamepad, Constants.Buttons.LST_AXS_RTRIGGER).whileTrue(new LowRotary(m_placementRotaryArm, m_placementExtensionArm));

    new POVButton(m_weaponsGamepad, Constants.Buttons.LST_POV_N).whileTrue(new ExtendExtension(m_placementExtensionArm, m_placementRotaryArm));
    new POVButton(m_weaponsGamepad, Constants.Buttons.LST_POV_S).whileTrue(new CollapseExtension(m_placementExtensionArm, m_placementRotaryArm));
    new POVButton(m_weaponsGamepad, Constants.Buttons.LST_POV_E).whileTrue(new IntermediateExtension(m_placementExtensionArm, m_placementRotaryArm));
    new POVButton(m_weaponsGamepad, Constants.Buttons.LST_POV_W).whileTrue(new ZeroExtension(m_placementExtensionArm));


    SmartDashboard.putData(new FlipFieldOriented(m_chassis));

    if (Constants.debugMode) {
      Shuffleboard.getTab("Test").add("Spin motor down", new zeroExtensionArm(m_placementExtensionArm));
    }
  }

  /**
   * Resets odometry to 0, 0, 0
   */
  public boolean resetOdometry() {
     OdoPosition positionToResetTo = m_limelight.calculate();
    if (positionToResetTo == null) {
      return false;
    }
    m_chassis.resetOdometry(positionToResetTo.getPosition());
    return true;
  }

  public void resetOdometryWithoutApril() {
    m_chassis.resetOdometry(new Pose2d(0, 0, new Rotation2d()));
  }


  /**
   * Gets the selected auton command that is on shuffleboard
   *
   * @return the auton routine
   */
  public Command getAutonCmd() {
    return m_autonManager.pick();
  }

  /**
   * Schedules a command to zero the extension arm
   */
  public void zeroCommand() {
    CommandScheduler.getInstance().schedule(new SequentialCommandGroup(new zeroExtensionArm(m_placementExtensionArm), new AutoZeroRotryArm(m_placementRotaryArm)));
  }

}
