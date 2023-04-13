// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.crypto.spec.RC5ParameterSpec;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Newman_Constants.Constants;
import frc.robot.supportingClasses.Gains.AccelerationManager;
import frc.robot.supportingClasses.Gains.VelocityGainFilter;
import frc.robot.subsystems.Chassis;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.supportingClasses.BoundingBox;

/**
 * The extension arm subsystem for the placement mechanism
 */
public class ExtensionArm extends SubsystemBase {
  /**
   * the motor controller for the motor on the extension arm winch
   */
  private final WPI_TalonFX extensionMotor;
  /**
   * limit switch which is at our 0 point for the extension arm
   */
  private final DigitalInput m_limitSwitch;
  private final Chassis m_chassis;

  /**
   * Holds the current setpoint of motion magic. Default setpoint is 0 which is retracted
   */
  private double currentSetpoint = 0;

  /**
   * Generic entry for updating the "P" value of the feedback controller on the extension arm
   */
  private final GenericEntry n_placementExtensionArmP;

  /**
   * Holds the last read "P" value from network tables. Is used in order to have fewer calls across CAN
   */
  private double l_placementExtensionArmP;

  /**
   * Generic entry for updating the "I" value of the feedback controller on the extension arm
   */
  private final GenericEntry n_placementExtensionArmI;
  private GenericEntry isInBox;

  /**
   * Holds the last read "I" value from network tables. Is used in order to have fewer calls across CAN
   */
  private double l_placementExtensionArmI;
  private GenericEntry n_placementExtensionArmD;
  private double m_placementExtensionArmD;
  private GenericEntry n_placementExtensionArmS_Strength;
  private int l_placementExtensionArmS_Strength;
  Pose3d armPos;

  private final double collapsedPosition = 0;
  private final double intermediatePosition = Constants.Extension.kMaxExtensionLength / 2;
  private final double extendedPosition = Constants.Extension.kMaxExtensionLength;

  /**
   * Generic entry for updating the "D" value of the feedback controller on the extension arm
   */

  /**
   * Holds the last read "D" value from network tables. Is used in order to have fewer calls across CAN
   */
  private double l_placementExtensionArmD;

  /**
   * Holds the current speed of the arm
   */
  public double armSpeed = 0;

  /**
   * The tolerance to determine if we are at the position we want to be on the extension arm
   */
  private final double positionDeadband = 10000;

  /**
   * A mechanism2D object which is used to draw on glass
   */
  protected MechanismLigament2d ligament;

  /**
   * The acceleration manager which is used to determine the velocity gain.
   * Shouldn't be used in comp.
   */
  private final AccelerationManager accelerationManager;

  /**
   * The Velocity gain filter. Should be used with {@link AccelerationManager}
   */
  protected final VelocityGainFilter gainFilter;

  /**
   * Initializes the extension arm and configures the necessary device settings.
   * Motors are set to: Factory default, then given 9 volts of voltage compensation, and put in brake mode
   *
   * @param ligament the ligament object that is on smart-dashboard
   */
  public ExtensionArm(MechanismLigament2d ligament, Chassis chassis) {
    extensionMotor = new WPI_TalonFX(Constants.CAN_ExtensionArm);
    extensionMotor.configFactoryDefault();
    extensionMotor.config_kP(0,Constants.Extension.kExtensionArmP);
    extensionMotor.config_kI(0,Constants.Extension.kExtensionArmI);
    extensionMotor.config_kD(0,Constants.Extension.kExtensionArmD);

    extensionMotor.setInverted(false);
    extensionMotor.setSensorPhase(false);

    extensionMotor.configMotionCruiseVelocity(Constants.Extension.kMaxVelocityPlacementExtensionArm);
    extensionMotor.configMotionAcceleration(Constants.Extension.kMaxAccelerationPlacementExtensionArm);
    extensionMotor.configMotionSCurveStrength(l_placementExtensionArmS_Strength);

    extensionMotor.setNeutralMode(NeutralMode.Brake);

    extensionMotor.configVoltageCompSaturation(Constants.kMaxExtensionArmVoltage);
    extensionMotor.enableVoltageCompensation(true);

    m_limitSwitch = new DigitalInput(Constants.PUNCHY_LIMIT_SWITCH);
    m_chassis = chassis;

    ShuffleboardTab placement = Shuffleboard.getTab("Extension Arm");
    n_placementExtensionArmP = placement.add("p", Constants.Extension.kExtensionArmP).getEntry();
    n_placementExtensionArmI = placement.add("i", Constants.Extension.kExtensionArmI).getEntry();
    n_placementExtensionArmD = placement.add("d", Constants.Extension.kExtensionArmD).getEntry();
    isInBox = placement.add("isInBox", false).getEntry();

    accelerationManager = new AccelerationManager();
    gainFilter = new VelocityGainFilter(9, "extension", this::getSpeedTicksPerSecond, accelerationManager);

    this.ligament = ligament;
  }

  private double getSmartSpeed(double y) {
    if (y < 0) {
      if (brokeLimit()) {
        resetEncoders();
        y = 0;
      }
    } else if (y > 0) {
      if (getPositionTicks() >= Math.abs(Constants.Extension.kMaxExtensionLength)) {
        y = 0;
      }
    }
    return y;
  }

  /**
   * If the Arm is retracting and hits the limit switch we reset encoder's and we stop the arm from
   * further retracting to prevent the motor from breaking the arm.
   * We also need to check if the arm is at max extension. If so then we stop the motors.
   * The ligament object on glass is also updated from here.
   * If the extension arm isn't running at the current desired speed then spin the arm
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ligament.setLength(getPositionMeters());
    double y = getSmartSpeed(armSpeed);
    if (y != armSpeed) {
      // It's updated so update the motor
      spinExtensionArm(y);
    }

    double r = getLengthExtensionArm() + 0.09525;
    Rotation3d armRotation = new Rotation3d();
    Translation3d armTranslation = new Translation3d(r * Math.cos(m_chassis.getYaw()), 0, r * Math.sin(m_chassis.getYaw()));
    armPos = new Pose3d(armTranslation, armRotation);
    BoundingBox[] boundingboxes = RobotContainer.getBoundingBoxes();
    for (int i = 0; i < boundingboxes.length; i++) {
      if (boundingboxes[i].boxBad(armPos)) {
        isInBox.setBoolean(true);
        if (m_chassis.getX() - (r) <= Constants.Field.xPositionForGridBlue || m_chassis.getX() + (r) >= Constants.Field.xPositionForGridRed) {
          if (y > 0) {
            y = 0;
          }
        }
      } else {
        isInBox.setBoolean(false);
      }
    }
  }


  /*
   * Extend the arm all the way out
   */
  public void extendArmFull() {
    currentSetpoint = extendedPosition;
    extensionMotor.set(ControlMode.MotionMagic, extendedPosition);
  }

  public Pose3d getArmPos() {
    return armPos;
  }

  /**
   * The intermediate position to extend the arm to
   */
  public void intermediateArm() {
    currentSetpoint = intermediatePosition;
    extensionMotor.set(ControlMode.MotionMagic, intermediatePosition);
  }

  /**
   * Updates {@link #currentSetpoint}.
   * Extends the arm to a passed in length.
   * Uses motion magic.
   * Should only be called once per routine.
   * Constant updates will result in a jerky on-off output to the motor.
   * @param length (ticks) to extend the arm to.
   */
  public void extendArmTo(double length) {
    currentSetpoint = length;
    extensionMotor.set(ControlMode.MotionMagic, length);
  }

  /**
   * Stops the arm.
   */
  public void stop() {
    extensionMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * give the extension a dumb power of 20% of {@link Constants#kMaxExtensionArmVoltage}
   */
  public void dumbPower() {
    extensionMotor.set(ControlMode.PercentOutput, 0.2);
  }

  /**
   * gets the position of the arm
   * @return the position of the extension arm in meters
   */
  public double getPositionMeters() {
    return Constants.Extension.kTicksToMetersExtension * extensionMotor.getSelectedSensorPosition() + Constants.Extension.kExtensionArmLengthRetractedMeters;
  }

  /**
   * gets the ticks that the extension motor has counted
   * @return the position of the extension arm in ticks. Max length should be {@link Constants.Extension#kMaxExtensionLength}
   */
  public double getPositionTicks() {
    return extensionMotor.getSelectedSensorPosition();
  }

  /**
   * Gets the speed of the extension arm in m/s
   * @return the speed of the extension arm in meters per second
   */
  public double getSpeedMetersPerSecond() {
    return 10 * Constants.Extension.kTicksToRadiansExtensionPlacement * extensionMotor.getSelectedSensorVelocity();
  }

  /**
   * Gets the speed of the extension arm in ticks
   * @return the speed of the mechanism in ticks per second
   */
  public double getSpeedTicksPerSecond() {
    return 10 * extensionMotor.getSelectedSensorVelocity();
  }

  /**
   * Uses the limit switch to see if we have broken it. Limit switch input is inverted (false is broken).
   * @return Whether the extension arm is at the limit switch
   */
  public boolean brokeLimit() {
    return !m_limitSwitch.get();
  }

  /**
   * update values on shuffleboard.
   * Values that are updated here are: P, I, and D values for the feedback controller.
   */
  public void updateValues() {
    if (l_placementExtensionArmP != n_placementExtensionArmP.getDouble(l_placementExtensionArmP)) {
      l_placementExtensionArmP = n_placementExtensionArmP.getDouble(l_placementExtensionArmP);
      extensionMotor.config_kP(0, l_placementExtensionArmP);
    }
    if (l_placementExtensionArmI != n_placementExtensionArmI.getDouble(l_placementExtensionArmI)){
      l_placementExtensionArmI = n_placementExtensionArmI.getDouble(l_placementExtensionArmI);
      extensionMotor.config_kI(0, l_placementExtensionArmI);
    }
    if (l_placementExtensionArmD != n_placementExtensionArmD.getDouble(l_placementExtensionArmD)){
      l_placementExtensionArmD = n_placementExtensionArmD.getDouble(l_placementExtensionArmD);
      extensionMotor.config_kD(0, l_placementExtensionArmD);
    }
  }

  /**
   * spins the extension arm.
   * calls {@link #getSmartSpeed(double)} in order to apply soft limits.
   * If we are in debug mode (based on {@link Constants#debugMode}) ten it updates the acceleration manager ({@link #accelerationManager}).
   * @param speed to run the motor at. Bounds: (-1 - 1) gets multiplied by {@link Constants#kMaxExtensionArmVoltage} and that is supplied to the motor.
   */
  public void spinExtensionArm(double speed) {
    speed = getSmartSpeed(speed);
    if (Constants.debugMode) {
      accelerationManager.update(getSpeedMetersPerSecond(), Timer.getFPGATimestamp());
    }
    extensionMotor.set(speed);
    armSpeed = speed;
  }

  public double getDistanceExtensionArm(){
    return Constants.Extension.kTicksToRadiansExtensionPlacement * extensionMotor.getSelectedSensorPosition();
  }

  /**
   * Reset the extension motor encoders
   */
  public void resetEncoders() {
    extensionMotor.setSelectedSensorPosition(0);
  }

  /**
   * Whether we are at the position or not in a dead band defined by {@link #positionDeadband}
   * @return whether the extension arm is at it's set position
   */
  public boolean atPosition() {
    return Math.abs(extensionMotor.getSelectedSensorPosition() - currentSetpoint) <= positionDeadband; //TODO: better is finished logic
  }

  /**
   * Extends the extension arm to the setpoint on the ground.
   * Uses motion magic.
   */

  public double getLengthExtensionArm(){
    return Constants.Extension.kTicksToMetersExtension * extensionMotor.getSelectedSensorPosition() + Constants.Extension.kExtensionArmLengthExtendedMeters;
  }

  public void extendArmToGroundForCone() {
    currentSetpoint = Constants.offGroundAngleCone;
    extensionMotor.set(ControlMode.MotionMagic, currentSetpoint);
  }


  /**
   * Initializes the sendable builder with entry's we want on shuffleboard
   * @param builder sendable builder
   */
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("extension length", this::getPositionTicks, null);
    builder.addDoubleProperty("Extension length", this::getPositionMeters, null);
  }
}
