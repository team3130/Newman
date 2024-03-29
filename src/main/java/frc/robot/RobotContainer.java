// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Newman_Constants.Constants;
import frc.robot.commands.Balance.Balance;
import frc.robot.commands.Balance.DeadReckonBalance;
import frc.robot.commands.Balance.OnToRamp;
import frc.robot.commands.Balance.RileyPark;
import frc.robot.commands.Balance.SearchBalance;
import frc.robot.commands.Chassis.FlipFieldOriented;
import frc.robot.commands.Chassis.TeleopDrive;
import frc.robot.commands.Chassis.ZeroEverything;
import frc.robot.commands.Chassis.ZeroWheels;
import frc.robot.commands.Chassis.presets.GoToClosestPlacementPosition;
import frc.robot.commands.Chassis.presets.GoToHumanPlayerStation;
import frc.robot.commands.Hopper.ReverseHopper;
import frc.robot.commands.Hopper.SpinHopper;
import frc.robot.commands.Intake.ToggleIntake;
import frc.robot.commands.Manipulator.ToggleManipulator;
import frc.robot.commands.Manipulator.UnClampManipulator;
import frc.robot.commands.Placement.AutoZeroExtensionArm;
import frc.robot.commands.Placement.AutoZeroRotryArm;
import frc.robot.commands.Placement.ManualControl.MoveExtensionArm;
import frc.robot.commands.Placement.ManualControl.MoveRotaryArm;
import frc.robot.commands.Placement.ToggleBrake;
import frc.robot.commands.Placement.presets.GoToHighScoring;
import frc.robot.commands.Placement.presets.GoToMidScoringCones;
import frc.robot.commands.Placement.presets.GoToMidScoringCube;
import frc.robot.commands.Placement.presets.GoToPickupOffGround;
import frc.robot.controls.JoystickTrigger;
import frc.robot.sensors.Limelight;
import frc.robot.sensors.Navx;
import frc.robot.subsystems.*;
import frc.robot.supportingClasses.Auton.AutonCommand;
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
  protected final AutonManager m_autonManager;

  /**
   * The driver Xbox controller object that we use to map buttons to commands.
   * The left joystick is used with a default command to drive the bot to a position.
   * The right joystick is used with a default command for the holonomic rotation of the bot.
   * For more button bindings see {@link #configureButtonBindings()}
   */
  private final XboxController m_driverGamepad;

  /**
   * The driver Xbox controller object that we use to map buttons to commands.
   * The left joystick is used with a default command for manually moving the extension arm.
   * The right joystick is used with a default command for manually moving the rotary arm.
   * For more button bindings see {@link #configureButtonBindings()}
   */
  private final XboxController m_weaponsGamepad;

  /**
   * The chassis singleton.
   * Is used to make commands that require the chassis subsystem.
   */
  private final Chassis m_chassis;

  /**
   * The extension arm singleton.
   * Is used for making commands that require the extension arm.
   */
  private final ExtensionArm m_extensionArm;

  /**
   * The rotary arm singleton.
   * Is used for making commands that require the rotary arm.
   */
  private final RotaryArm m_rotaryArm;

  /**
   * The manipulator singleton.
   * Is used for making commands that require the manipulator.
   */
  private final Manipulator m_manipulator = new Manipulator();

  /**
   * The hopper singleton.
   * Is used for making commands that require the hopper.
   */
  private final Hopper m_hopper;

  /**
   * The intake singleton.
   * Is used for making commands that require the intake.
   */
  private final Intake m_intake;

  /**
   * The limelight singleton
   * Is used for updating the robots position using april tags.
   */
  private final Limelight m_limelight;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    m_driverGamepad = new XboxController(0);
    m_weaponsGamepad = new XboxController(1);

    m_limelight = new Limelight();

    m_chassis = new Chassis(m_limelight);
    m_hopper = new Hopper();
    m_intake = new Intake();

    Mechanism2d arm = new Mechanism2d(3, 3.5);
    MechanismRoot2d root = arm.getRoot("arm", 0.5, 2);
    MechanismLigament2d zero = new MechanismLigament2d("retracted", Constants.Extension.kExtensionArmLengthRetractedMeters, -90);
    MechanismLigament2d limit = new MechanismLigament2d(
            "limit",
            Constants.Extension.kExtensionArmLengthExtendedMeters,
            Math.toDegrees(Constants.highPosition) - 90,
            6,
            new Color8Bit(0, 0, 255));
    root.append(zero);
    root.append(limit);

    SendableRegistry.add(arm, "arm");
    SmartDashboard.putData(arm);
    
    m_extensionArm = new ExtensionArm(zero);
    m_rotaryArm = new RotaryArm(zero);

    m_autonManager = new AutonManager(m_chassis, m_intake, m_rotaryArm, m_extensionArm, m_manipulator);

    m_chassis.setDefaultCommand(new TeleopDrive(m_chassis, m_driverGamepad));

    m_rotaryArm.setDefaultCommand(new MoveRotaryArm(m_rotaryArm, m_extensionArm, m_weaponsGamepad));
    m_extensionArm.setDefaultCommand(new MoveExtensionArm(m_extensionArm, m_weaponsGamepad));

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
      tab.add(m_extensionArm);
      tab.add(m_rotaryArm);
      tab.add(m_manipulator);
      tab.add(m_hopper);
      tab.add(m_intake);
      m_chassis.shuffleboardVom(Shuffleboard.getTab("Swerve Modules"));
    }
  }

  /**
   * This shouldn't be necessary as we can just pass the initialized object,
   * However this can be here just in case we need it last minute
   *
   * @return the driver gamepad
   */
  public XboxController getDriverGamepad() {
    return m_driverGamepad;
  }

  /**
   * This shouldn't be necessary as we can just pass the initialized object,
   * However this can be here just in case we need it last minute
   *
   * @return the weapons game pad
   */
  public XboxController getWeaponsGamepad() {
    return m_weaponsGamepad;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //Driver Gamepad:
    //Intake
    //new JoystickButton(m_driverGamepad, Constants.Buttons.LST_BTN_RBUMPER).onTrue(new ToggleIntake(m_intake));
    new JoystickButton(m_driverGamepad, Constants.Buttons.LST_BTN_RBUMPER).whileTrue(new ToggleIntake(m_intake));

    //Chassis
    new JoystickButton(m_driverGamepad, Constants.Buttons.LST_BTN_B).whileTrue(new FlipFieldOriented(m_chassis));
    new POVButton(m_driverGamepad, Constants.Buttons.LST_POV_N).whileTrue(new ZeroEverything(m_chassis));
    new POVButton(m_driverGamepad, Constants.Buttons.LST_POV_W).whileTrue(new ZeroWheels(m_chassis));
    //new JoystickButton(m_driverGamepad, Constants.Buttons.LST_BTN_X).whileTrue(new Balancing(m_chassis));

    if (Constants.debugMode) {
      new JoystickButton(m_driverGamepad, Constants.Buttons.LST_BTN_RJOYSTICKPRESS).onTrue(new SequentialCommandGroup(new OnToRamp(m_chassis, false), new Balance(m_chassis), new RileyPark(m_chassis)));
      new JoystickButton(m_driverGamepad, Constants.Buttons.LST_BTN_Y).onTrue(new SequentialCommandGroup(new SearchBalance(m_chassis), new RileyPark(m_chassis)));
      new JoystickTrigger(m_driverGamepad, Constants.Buttons.LST_AXS_RTRIGGER).whileTrue(new GoToClosestPlacementPosition(m_chassis, m_autonManager));
      new JoystickTrigger(m_driverGamepad, Constants.Buttons.LST_AXS_LTRIGGER).whileTrue(new GoToHumanPlayerStation(m_chassis, m_autonManager));
    }

    //Weapons Gamepad:

    //Intake
    new JoystickButton(m_weaponsGamepad, Constants.Buttons.LST_BTN_Y).whileTrue(new SpinHopper(m_hopper));
/*    new POVButton(m_weaponsGamepad, Constants.Buttons.LST_POV_S).whileTrue(new ReverseHopper(m_hopper, m_pivot));
    new POVButton(m_weaponsGamepad, Constants.Buttons.LST_POV_N).whileTrue(new UnjamHopper(m_hopper));*/

    
    //Placement
/*    new JoystickButton(m_weaponsGamepad, Constants.Buttons.LST_BTN_B).whileTrue(
            new SequentialCommandGroup(
                    new ToggleGrabber(m_manipulator),
                    new AutoZeroExtensionArm(m_extensionArm),
                    new AutoZeroRotryArm(m_rotaryArm),
                    new GoToMidScoringCube(m_rotaryArm, m_extensionArm)
    ));*/

    new JoystickButton(m_weaponsGamepad, Constants.Buttons.LST_BTN_RBUMPER).onTrue(new ToggleBrake(m_rotaryArm));
    new JoystickTrigger(m_weaponsGamepad, Constants.Buttons.LST_AXS_LTRIGGER).onTrue(new ToggleManipulator(m_manipulator));
/*    new POVButton(m_weaponsGamepad, Constants.Buttons.LST_POV_W).whileTrue(new AutoZeroExtensionArm(m_ExtensionArm));
    new POVButton(m_weaponsGamepad, Constants.Buttons.LST_POV_E).whileTrue(new AutoZeroRotryArm(m_RotaryArm));*/
    // new POVButton(m_weaponsGamepad, Constants.Buttons.LST_POV_N).whileTrue(new GoToHighScoring(m_rotaryArm, m_extensionArm));
    // new POVButton(m_weaponsGamepad, Constants.Buttons.LST_POV_N).whileTrue(new GoToPickupCube(m_rotaryArm, m_extensionArm));
    // new POVButton(m_weaponsGamepad, Constants.Buttons.LST_POV_E).whileTrue(new GoToMidScoringCones(m_rotaryArm, m_extensionArm));
//    new POVButton(m_weaponsGamepad, Constants.Buttons.LST_POV_W).whileTrue(new GoToPickupOffGround(m_rotaryArm, m_extensionArm));
    // new POVButton(m_weaponsGamepad, Constants.Buttons.LST_POV_W).whileTrue(new GoToPickupOffGround(m_rotaryArm, m_extensionArm));
//    new POVButton(m_weaponsGamepad, Constants.Buttons.LST_POV_W).whileTrue(new GoToLowScoring(m_rotaryArm, m_extensionArm));
    // new POVButton(m_weaponsGamepad, Constants.Buttons.LST_POV_S).whileTrue(new GoToMidScoringCube(m_rotaryArm, m_extensionArm));
    new JoystickButton(m_weaponsGamepad, Constants.Buttons.LST_BTN_LBUMPER).whileTrue(new SequentialCommandGroup(
            new AutoZeroExtensionArm(m_extensionArm),
            new AutoZeroRotryArm(m_rotaryArm))
    );

    new JoystickButton(m_weaponsGamepad, Constants.Buttons.LST_BTN_B).whileTrue(new ReverseHopper(m_hopper));

    SmartDashboard.putData(new FlipFieldOriented(m_chassis));

  }

  /**
   * Gets the selected auton command that is on shuffleboard
   *
   * @return the auton routine
   */
  public CommandBase getAutonCmd() {
    return m_autonManager.pick();
  }

  public CommandBase resetEverything() {
    return new ZeroEverything(m_chassis);
  }

  /**
   * Schedules a command to zero the extension arm
   */
  public CommandBase zeroCommand() {
    return
            new SequentialCommandGroup(
                    new AutoZeroExtensionArm(m_extensionArm),
                    new AutoZeroRotryArm(m_rotaryArm)
            );
  }

  /**
   * Robot container periodic method.
   * Needs to be called from {@link Robot#robotPeriodic()} in order to function properly.
   */
  public void periodic() {}

  /**
   * Makes a command to unclamp the manipulator. Should be used on teleop init to make sure that we don't enable and zero with manipulator clamped.
   * @return the instant command to unclamp manipulator
   */
  public CommandBase unClampManipulator() {
    return new UnClampManipulator(m_manipulator);
  }

  /**
   * Resets odometry to the position of the april tag
   * @return success or not
   */
  public boolean resetOdometryWithAprilTag() {
    OdoPosition position = m_limelight.calculate();
    if (position != null) {
      m_chassis.resetOdometry(position.getPosition());
      return true;
    }
    return false;
  }

  /**
   * Resets odometry without april tags to 0, 0, 0.
   * This is needed because the absolute encoders don't turn on for a while.
   * The logic in Robot.Java should make it so that this can't get ran periodically
   */
  public void resetOdometryWithoutApril() {
    m_chassis.resetOdometry(new Pose2d(0, 0, new Rotation2d()));
  }

  /**
   * Updates the chassis position periodically.
   * Calls {@link Chassis#updateOdometery()}
   */
  public void updateChassisPose() {
    m_chassis.updateOdometery();
  }

  /**
   * Packages auton commands so that they go to the start of the command before running the main routines.
   * As of 4/13 this command also brings up the rotary arm to high.
   * If mainPath is an auton command then we can convert it to an AutonCommand and go to the start of its path. 
   * If it is not an Auton command then this method will just return the passed in command.
   * @param mainPath the main path to run. 
   * @return A full auton routine if the passed in command is an auton command. Otherwise it will just return the passed in command.
   */
  public CommandBase packageAuton(CommandBase mainPath) {
    try {
      return m_autonManager.goToStartOfCommand((AutonCommand) mainPath);
    }
    catch (Exception ignored) {
      return mainPath;
    }
  }

}
