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
  private double extensionArmSpeed = 1;

  private double currentSetpoint = 0;

  /**
   * Network table variables
   */
  private ShuffleboardTab Placement;
  private GenericEntry n_placementExtensionArmP;
  private double l_placementExtensionArmP;
  private GenericEntry n_placementExtensionArmI;
  private double l_placementExtensionArmI;
  private GenericEntry n_placementExtensionArmD;
  private double l_placementExtensionArmD;
  private GenericEntry n_placementExtensionArmS_Strength;
  private double l_placementExtensionArmS_Strength;

  private final double collapsedPosition = 0;
  private final double intermediatePosition = Constants.kMaxExtensionLength / 2;
  private final double extendedPosition = Constants.kMaxExtensionLength;

  public double armSpeed = 0;
  public int sStrengthPlacementExtensionArm = 0;
  private final double positionDeadband = 1000;

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
    extensionMotor.config_kP(0,Constants.kExtensionArmP);
    extensionMotor.config_kI(0,Constants.kExtensionArmI);
    extensionMotor.config_kD(0,Constants.kExtensionArmD);

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
    n_placementExtensionArmP = Placement.add("p", Constants.kExtensionArmP).getEntry();
    n_placementExtensionArmI = Placement.add("i", Constants.kExtensionArmI).getEntry();
    n_placementExtensionArmD = Placement.add("d", Constants.kExtensionArmD).getEntry();

    n_placementExtensionArmS_Strength = Placement.add("s strength", sStrengthPlacementExtensionArm).getEntry();

    accelerationManager = new AccelerationManager();
    gainFilter = new VelocityGainFilter(9, "extension", this::getSpeedTicksPerSecond, accelerationManager);

    this.ligament = ligament;
  }

  private double getSmartSpeed(double y){
    if (y < 0) {
      if (brokeLimit()) {
        resetEncoders();
        y = 0;
      }
    } else if (y > 0) {
      if (getPositionTicks() >= Math.abs(Constants.kMaxExtensionLength)) {
        y = 0;
      }
    }
    return y;
  }

  /* If the Arm is retracting and hits the limit switch we reset encoder's and
  stop the arm from further retracting to prevent the motor from breaking the arm.
  We also need to check if the arm is at max extension. If so then we stop the motors*/
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ligament.setLength(getLengthExtensionArm());
    double y = getSmartSpeed(armSpeed);
    if (y != armSpeed) {
      // It's updated so update the motor
      spinExtensionArm(y);
    }
  }


  /*
   * Extend the arm all the way out
   */
  public void extendArmFull() {
    currentSetpoint = extendedPosition;
    extensionMotor.set(ControlMode.MotionMagic, extendedPosition);
  }

  /**
   * The intermediate position to extend the arm to
   */
  public void intermediateArm() {
    currentSetpoint = intermediatePosition;
    extensionMotor.set(ControlMode.MotionMagic, intermediatePosition);
  }

  /**
   * collapse the arm, can be replaced with zero?
   */
  public void collapseArm() {
    currentSetpoint = collapsedPosition;
    extensionMotor.set(ControlMode.MotionMagic, collapsedPosition);
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
    if (l_placementExtensionArmP != n_placementExtensionArmP.getDouble(Constants.kExtensionArmP)){
      l_placementExtensionArmP = Constants.kExtensionArmP;
      extensionMotor.config_kP(0, l_placementExtensionArmP);
    }
    if (l_placementExtensionArmI != n_placementExtensionArmI.getDouble(Constants.kExtensionArmI)){
            l_placementExtensionArmI = Constants.kExtensionArmI;
      extensionMotor.config_kI(0, l_placementExtensionArmI);
    }
    if (l_placementExtensionArmD != n_placementExtensionArmD.getDouble(Constants.kExtensionArmD)){
      l_placementExtensionArmD = Constants.kExtensionArmD;
      extensionMotor.config_kD(0, l_placementExtensionArmD);
    }
    if (l_placementExtensionArmS_Strength != n_placementExtensionArmS_Strength.getDouble(sStrengthPlacementExtensionArm)){
      l_placementExtensionArmS_Strength = n_placementExtensionArmS_Strength.getDouble(sStrengthPlacementExtensionArm);
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
    scalar = getSmartSpeed(scalar);
    if (Constants.debugMode) {
      accelerationManager.update(getSpeedMetersPerSecond(), Timer.getFPGATimestamp());
    }
    extensionMotor.set(scalar);
    armSpeed = scalar;
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
    return extensionMotor.isMotionProfileFinished() && Math.abs(extensionMotor.getSelectedSensorPosition() - currentSetpoint) <= positionDeadband; //TODO: better is finished logic
  }

  /**
   * @return the length of the extension arm
   */
  public double getLengthExtensionArm(){
    return Constants.kTicksToMetersExtension * extensionMotor.getSelectedSensorPosition() + Constants.kExtensionArmLengthExtended;
  }

}
