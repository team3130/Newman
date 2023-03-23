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
import frc.robot.commands.Chassis.TeleopDrive;
import frc.robot.commands.Chassis.ZeroEverything;
import frc.robot.commands.Chassis.ZeroWheels;
import frc.robot.commands.Hopper.ReverseHopper;
import frc.robot.commands.Hopper.SpinHopper;
import frc.robot.commands.Hopper.UnjamHopper;
import frc.robot.commands.Intake.ToggleIntake;
import frc.robot.commands.Manipulator.ToggleGrabber;
import frc.robot.commands.Placement.*;
import frc.robot.commands.Placement.ManualControl.MoveExtensionArm;
import frc.robot.commands.Placement.ManualControl.MoveRotaryArm;
import frc.robot.commands.Placement.presets.GoToMidScoring;
import frc.robot.controls.JoystickTrigger;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.*;
import frc.robot.supportingClasses.Auton.AutonManager;
import frc.robot.supportingClasses.Vision.OdoPosition;

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
  private final ExtensionArm m_ExtensionArm = new ExtensionArm();
  private final RotaryArm m_RotaryArm = new RotaryArm();
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

    m_RotaryArm.setDefaultCommand(new MoveRotaryArm(m_RotaryArm, m_ExtensionArm, m_weaponsGamepad));
    m_ExtensionArm.setDefaultCommand(new MoveExtensionArm(m_ExtensionArm, m_weaponsGamepad));


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
      tab.add(m_ExtensionArm);
      tab.add(m_RotaryArm);
      tab.add(m_manipulator);
      tab.add(m_hopper);
      tab.add(m_pivot);
      m_chassis.shuffleboardVom(Shuffleboard.getTab("Swerve Modules"));
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
    new JoystickButton(m_weaponsGamepad, Constants.Buttons.LST_BTN_B).whileTrue(
            new SequentialCommandGroup(
                    new ToggleGrabber(m_manipulator),
                    new AutoZeroExtensionArm(m_ExtensionArm),
                    new AutoZeroRotryArm(m_RotaryArm),
                    new GoToMidScoring(m_RotaryArm, m_ExtensionArm),
                    new IntermediateExtension(m_ExtensionArm, m_RotaryArm))
    );

    //Driver Gamepad:
    //Intake
    new JoystickButton(m_driverGamepad, Constants.Buttons.LST_BTN_RBUMPER).onTrue(new ToggleIntake(m_pivot));

    //Chassis
    new JoystickButton(m_driverGamepad, Constants.Buttons.LST_BTN_B).whileTrue(new FlipFieldOriented(m_chassis));
    new POVButton(m_driverGamepad, Constants.Buttons.LST_POV_N).whileTrue(new ZeroEverything(m_chassis));
    new POVButton(m_driverGamepad, Constants.Buttons.LST_POV_W).whileTrue(new ZeroWheels(m_chassis));
    //new JoystickButton(m_driverGamepad, Constants.Buttons.LST_BTN_X).whileTrue(new Balancing(m_chassis));


    //Weapons Gamepad:

    //Intake
    new JoystickButton(m_weaponsGamepad, Constants.Buttons.LST_BTN_Y).whileTrue(new SpinHopper(m_hopper));
    new POVButton(m_weaponsGamepad, Constants.Buttons.LST_POV_S).whileTrue(new ReverseHopper(m_hopper, m_pivot));
    new POVButton(m_weaponsGamepad, Constants.Buttons.LST_POV_N).whileTrue(new UnjamHopper(m_hopper));

    //Placement
    new JoystickButton(m_weaponsGamepad, Constants.Buttons.LST_BTN_RBUMPER).onTrue(new ToggleBrake(m_RotaryArm));
    new JoystickTrigger(m_weaponsGamepad, Constants.Buttons.LST_AXS_LTRIGGER).onTrue(new ToggleGrabber(m_manipulator));
    new POVButton(m_weaponsGamepad, Constants.Buttons.LST_POV_W).whileTrue(new AutoZeroExtensionArm(m_ExtensionArm));
    new POVButton(m_weaponsGamepad, Constants.Buttons.LST_POV_E).whileTrue(new AutoZeroRotryArm(m_RotaryArm));
    new JoystickButton(m_weaponsGamepad, Constants.Buttons.LST_BTN_LBUMPER).whileTrue(new SequentialCommandGroup(
            new AutoZeroExtensionArm(m_ExtensionArm),
            new AutoZeroRotryArm(m_RotaryArm))
    );

    new JoystickButton(m_weaponsGamepad, Constants.Buttons.LST_BTN_A).whileTrue(new ExtendExtension(m_ExtensionArm));

    /*
    new JoystickButton(m_weaponsGamepad, Constants.Buttons.LST_BTN_LBUMPER).whileTrue(new GoToHighSchoring(m_RotaryArm, m_ExtensionArm));
    new JoystickTrigger(m_weaponsGamepad, Constants.Buttons.LST_AXS_LTRIGGER).whileTrue(new GoToMidScoring(m_RotaryArm, m_ExtensionArm));
    new JoystickTrigger(m_weaponsGamepad, Constants.Buttons.LST_AXS_RTRIGGER).whileTrue(new GoToLowScoring(m_RotaryArm, m_ExtensionArm));
    */

    /*
    new POVButton(m_weaponsGamepad, Constants.Buttons.LST_POV_N).whileTrue(new ExtendExtension(m_ExtensionArm, m_RotaryArm));
    new POVButton(m_weaponsGamepad, Constants.Buttons.LST_POV_S).whileTrue(new CollapseExtension(m_ExtensionArm, m_RotaryArm));
    new POVButton(m_weaponsGamepad, Constants.Buttons.LST_POV_E).whileTrue(new IntermediateExtension(m_ExtensionArm, m_RotaryArm));
    */

    SmartDashboard.putData(new FlipFieldOriented(m_chassis));

    if (Constants.debugMode) {
      Shuffleboard.getTab("Test").add("Spin motor down", new AutoZeroExtensionArm(m_ExtensionArm));
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
    CommandScheduler.getInstance().schedule(
            new SequentialCommandGroup(
                    new AutoZeroExtensionArm(m_ExtensionArm),
                    new AutoZeroRotryArm(m_RotaryArm)
            ));
  }

}
