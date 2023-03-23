// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;
import frc.robot.supportingClasses.Gains.AccelerationManager;
import frc.robot.supportingClasses.Gains.VelocityGainFilter;

/**
 * The extension arm subsystem for the placement mechanism
 */
public class ExtensionArm extends SubsystemBase {
  // the motor controller
  private final WPI_TalonFX extensionMotor;
  // limit switch which is at our 0 point for the extension arm
  private final DigitalInput m_limitSwitch;

  protected MechanismLigament2d ligament;

  // the acceleration manager
  private final AccelerationManager accelerationManager;

  /**
   * Speed to run the motor at by default, can be changed in shuffleboard
   */
  private static double extensionArmSpeed = 1;


  /**
   * Network table variables
   */
  public ShuffleboardTab Placement;
  public GenericEntry n_placementExtensionArmP;
  public double l_placementExtensionArmP;
  public GenericEntry n_placementExtensionArmI;
  public double l_placementExtensionArmI;
  public GenericEntry n_placementExtensionArmD;
  public double l_placementExtensionArmD;
  public GenericEntry n_placementExtensionArmS_Strength;
  public double l_placementExtensionArmS_Strength;

  public final GenericEntry n_collapsedPosition;
  public final GenericEntry n_intermediatePosition;
  public final GenericEntry n_extendedPosition;
  public double collapsedPosition = 0;
  public double intermediatePosition = 1;
  public double extendedPosition = 175000;

  /**
   * The PID values for the extension arm controller
   */
  public double placementExtensionArmP = 1;
  public double placementExtensionArmI = 0;
  public double placementExtensionArmD = 0.05;

  public int sStrengthPlacementExtensionArm = 0;

  protected final VelocityGainFilter gainFilter;

  /**
   * Initializes the extension arm and configures the necessary device settings.
   * Motors are set to: Factory default, then given 9 volts of voltage compensation, and put in brake mode
   *
   * @param ligament the ligament object that is on smart-dashboard
   */
  public ExtensionArm(MechanismLigament2d ligament) {
    extensionMotor = new WPI_TalonFX(Constants.CAN_ExtensionArm);
    extensionMotor.configFactoryDefault();
    extensionMotor.config_kP(0,placementExtensionArmP);
    extensionMotor.config_kI(0,placementExtensionArmI);
    extensionMotor.config_kD(0,placementExtensionArmD);

    extensionMotor.setInverted(false);
    extensionMotor.setSensorPhase(false);

    extensionMotor.configMotionCruiseVelocity(Constants.kMaxVelocityPlacementExtensionArm);
    extensionMotor.configMotionAcceleration(Constants.kMaxAccelerationPlacementExtensionArm);
    extensionMotor.configMotionSCurveStrength(sStrengthPlacementExtensionArm);

    extensionMotor.setNeutralMode(NeutralMode.Brake);

    extensionMotor.configVoltageCompSaturation(Constants.kMaxExtensionArmVoltage);
    extensionMotor.enableVoltageCompensation(true);

    m_limitSwitch = new DigitalInput(Constants.PUNCHY_LIMIT_SWITCH);

    Placement = Shuffleboard.getTab("Extension Arm");
    n_placementExtensionArmP = Placement.add("p", placementExtensionArmP).getEntry();
    n_placementExtensionArmI = Placement.add("i", placementExtensionArmI).getEntry();
    n_placementExtensionArmD = Placement.add("d", placementExtensionArmD).getEntry();

    n_collapsedPosition = Placement.add("collapses position", collapsedPosition).getEntry();
    n_intermediatePosition = Placement.add("intermediate positon", intermediatePosition).getEntry();
    n_extendedPosition = Placement.add("extended position", extendedPosition).getEntry();

    n_placementExtensionArmS_Strength = Placement.add("s strength", sStrengthPlacementExtensionArm).getEntry();

    accelerationManager = new AccelerationManager();
    gainFilter = new VelocityGainFilter(9, "extension", this::getSpeedTicksPerSecond, accelerationManager);

    this.ligament = ligament;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ligament.setLength(getLengthExtensionArm());
  }

  /**
   * Extend the arm all the way out
   */
  public void extendArmFull() {
    extensionMotor.set(ControlMode.MotionMagic, extendedPosition);
  }

  /**
   * The intermediate position to extend the arm to
   */
  public void intermediateArm() {
    extensionMotor.set(ControlMode.MotionMagic, n_intermediatePosition.getDouble(intermediatePosition));
  }

  /**
   * collapse the arm, can be replaced with zero?
   */
  public void collapseArm() {
    extensionMotor.set(ControlMode.MotionMagic, n_collapsedPosition.getDouble(collapsedPosition));
  }

  /**
   * Stop the arm
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
   * @return the position of the extension arm in meters
   */
  public double getPositionMeters() {
    return Constants.kTicksToMetersExtension * extensionMotor.getSelectedSensorPosition();
  }

  /**
   * @return the position of the extension arm in ticks. Max length should be {@link Constants#kMaxExtensionLength}
   */
  public double getPositionTicks() {
    return extensionMotor.getSelectedSensorPosition();
  }

  /**
   * @return the speed of the extension arm in meters per second
   */
  public double getSpeedMetersPerSecond() {
    return 10 * Constants.kTicksToRadiansExtensionPlacement * extensionMotor.getSelectedSensorVelocity();
  }

  /**
   * @return the speed of the mechanism in ticks per second
   */
  public double getSpeedTicksPerSecond() {
    return 10 * extensionMotor.getSelectedSensorVelocity();
  }

  /**
   * @return Whether the extension arm is at the limit switch
   */
  public boolean brokeLimit() {
    return !m_limitSwitch.get();
  }

  /**
   * update values on shuffleboard
   */
  public void updateValues() {
    if (l_placementExtensionArmP != n_placementExtensionArmP.getDouble(placementExtensionArmP)){
      extensionMotor.config_kP(0, n_placementExtensionArmP.getDouble(placementExtensionArmP));
    }
    if (l_placementExtensionArmI != n_placementExtensionArmI.getDouble(placementExtensionArmI)){
      extensionMotor.config_kI(0, n_placementExtensionArmI.getDouble(placementExtensionArmI));
    }
    if (l_placementExtensionArmD != n_placementExtensionArmD.getDouble(placementExtensionArmD)){
      extensionMotor.config_kD(0, n_placementExtensionArmD.getDouble(placementExtensionArmD));
    }
    if (l_placementExtensionArmS_Strength != n_placementExtensionArmS_Strength.getDouble(sStrengthPlacementExtensionArm)){
      extensionMotor.configMotionSCurveStrength(0, (int) n_placementExtensionArmS_Strength.getDouble(sStrengthPlacementExtensionArm));
    }
  }

  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("extension length", this::getPositionTicks, null);
    builder.addDoubleProperty("Extension length", this::getPositionMeters, null);
  }

  /**
   * spins the extension arm
   * @param scalar to scale the output speed
   */
  public void spinExtensionArm(double scalar) {
    if (Constants.debugMode) {
      accelerationManager.update(getSpeedMetersPerSecond(), Timer.getFPGATimestamp());
    }
    extensionMotor.set(scalar);
  }

  /**
   * This method will be called once per scheduler run during simulation
   */
  @Override
  public void simulationPeriodic() {}

  /**
   * returns the speed we are currently running the motor at.
   * @return the control speed of the motor
   */
  public double getSpeed() {
    return extensionArmSpeed;
  }

  /**
   * Setter for the speed
   * @param newSpeed speed to set the arm to when we run it
   */
  public void updateSpeed(double newSpeed) {
    extensionArmSpeed = newSpeed;
  }

  /**
   * Reset the extension motor encoders
   */
  public void resetEncoders() {
    extensionMotor.setSelectedSensorPosition(0);
  }

  public boolean atPosition() {
    return extensionMotor.isMotionProfileFinished() && extensionMotor.getSelectedSensorPosition() == Constants.kMaxExtensionLength; //TODO: better is finished logic
  }

  /**
   * @return the length of the extension arm
   */
  public double getLengthExtensionArm(){
    return Constants.kTicksToMetersExtension * extensionMotor.getSelectedSensorPosition() + Constants.kExtensionArmLengthExtended;
  }
}
