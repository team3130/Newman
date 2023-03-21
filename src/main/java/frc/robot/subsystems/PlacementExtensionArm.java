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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;

public class PlacementExtensionArm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final WPI_TalonFX extensionMotor;
  private final DigitalInput m_limitSwitch;

    /**
   * Speed to run the motor at by default, can be changed in shuffleboard
   */
  private static double extensionArmSpeed = 1;


  //general
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
  public double extendedPosition = 2;

  public double placementExtensionArmP = 5.12295e-5 / 2;
  public double placementExtensionArmI = 0;
  public double placementExtensionArmD = 0;

  public int sStrengthPlacementExtensionArm = 0;

  public PlacementExtensionArm() {
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

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void extendArm() {
    extensionMotor.set(ControlMode.MotionMagic, n_extendedPosition.getDouble(n_extendedPosition.getDouble(extendedPosition)));
  }

  public void intermediateArm() {
    extensionMotor.set(ControlMode.MotionMagic, n_intermediatePosition.getDouble(n_intermediatePosition.getDouble(intermediatePosition)));
  }

  public void collapseArm() {
    extensionMotor.set(ControlMode.MotionMagic, n_collapsedPosition.getDouble(n_collapsedPosition.getDouble(collapsedPosition)));
  }

  public void stopArm() {
    extensionMotor.set(ControlMode.PercentOutput, 0);
  }

  public void dumbPower() {
    extensionMotor.set(ControlMode.PercentOutput, 0.2);
  }

  public boolean outsideBumper(PlacementRotaryArm rotaryArm) {
    return rotaryArm.getPositionPlacementArmAngle() > Math.toRadians(30);
  }

  public boolean wayOutsideBumper(PlacementRotaryArm rotaryArm) {
    return rotaryArm.getPositionPlacementArmAngle() > Math.toRadians(40);
  }

  public boolean isMoving(PlacementRotaryArm rotaryArm) { // alternative to passedBumper
    return !rotaryArm.isStationary();
  }

  public double getPositionPlacementArm() {
    return Constants.kTicksToMetersExtension * extensionMotor.getSelectedSensorPosition();
  }

  public double getSpeedPlacementArm() {
    return 10 * Constants.kTicksToRadiansExtensionPlacement * extensionMotor.getSelectedSensorVelocity();
  }

  public boolean brokeLimit() {
    return !m_limitSwitch.get();
  }

  public void updateValues(){
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
    builder.addDoubleProperty("extension length", this::getRawTicks, null);
    builder.addDoubleProperty("Extension length", this::getPositionPlacementArm, null);
  }

  public double getRawTicks() {
    return extensionMotor.getSelectedSensorPosition();
  }

    /**
   * spins the extension arm
   * @param scalar to scale the output speed
   */
  public void spinExtensionArm(double scalar) {
    extensionMotor.set(scalar);
  }

  /**
   * This method will be called once per scheduler run during simulation
   */
  @Override
  public void simulationPeriodic() {}


  /**
   * Stops the devices connected to this subsystem
   */
  public void stop() {
    extensionMotor.set(0);
  }

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

  public void resetEncoders() {
    extensionMotor.setSelectedSensorPosition(0);
  }

  public void RumbleFullPower(Joystick gamepad){
    gamepad.setRumble(GenericHID.RumbleType.kBothRumble,1);
  }

  public double getPositionPlacementArmExtensionRaw() {
    return extensionMotor.getSelectedSensorPosition();
  }
}
