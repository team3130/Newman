// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;

import java.util.HashMap;

public class RotaryArm extends SubsystemBase {

  public enum Position {
    ZERO, LOW, MID, HIGH
  }

  /** Creates a new ExampleSubsystem. */
  private final WPI_TalonFX rotaryMotor;
  private final Solenoid brake;
  private final boolean defaultState = true;
  private final double zeroPosition = 0;
  private final double lowPosition = Math.PI / 6;
  private final double midPosition = Math.PI / 4;
  private final double highPosition = Math.toRadians(100);
  private final DigitalInput limitSwitch;

  protected MechanismLigament2d ligament;
  private double outputSpeed = 0.6; // the speed we will run the rotary arm at

  private final long deadband = 100;

  private final HashMap<Position, Double> positionMap;

  private double placementRotaryArmP = 0.6;
  private double placementRotaryArmI = 0;
  private double placementRotaryArmD = 0.05;

  //private double placementRotaryArmFDown = 0;
  //private double placementRotaryArmFUp = 0;
  //private int sStrengthRotaryPlacementArm = 0;
  private ShuffleboardTab Placement;
  private GenericEntry n_placementRotaryArmP;
  private double l_placementRotaryArmP;
  private GenericEntry n_placementRotaryArmI;
  private double l_placementRotaryArmI;
  private GenericEntry n_placementRotaryArmD;
  private double l_placementRotaryArmD;

  private ShuffleboardTab n_TestTab = Shuffleboard.getTab("Test");

  /*private GenericEntry n_placementRotaryArmFUp;
  private double l_placementRotaryArmFUp;
  private GenericEntry n_placementRotaryArmFDown;
  private double l_placementRotaryArmFDown;
   */


  public static final TrapezoidProfile.Constraints rotaryArmConstraints = new TrapezoidProfile.Constraints(
          Constants.kMaxVelocityRotaryPlacementArm, Constants.kMaxAccelerationRotaryPlacementArm);
  /*
  private TrapezoidProfile lower = new TrapezoidProfile(rotaryArmConstraints,
          new TrapezoidProfile.State(0, 0),
          new TrapezoidProfile.State(lowPosition, 0));

  private TrapezoidProfile mid = new TrapezoidProfile(rotaryArmConstraints,
          new TrapezoidProfile.State(0, 0),
          new TrapezoidProfile.State(midPosition, 0));

  private TrapezoidProfile upper = new TrapezoidProfile(rotaryArmConstraints,
          new TrapezoidProfile.State(0, 0),
          new TrapezoidProfile.State(highPosition, 0));
          */

  // the profiled pid controller for rotary arm
  public ProfiledPIDController rotaryPID = new ProfiledPIDController(placementRotaryArmP, placementRotaryArmI,
          placementRotaryArmD, rotaryArmConstraints);


  public RotaryArm(MechanismLigament2d ligament) {
    rotaryMotor = new WPI_TalonFX(Constants.CAN_RotaryArm);
    rotaryMotor.configFactoryDefault();
    brake = new Solenoid(Constants.CAN_PNM, PneumaticsModuleType.CTREPCM, Constants.PNM_Brake);
    brake.set(defaultState);
    //rotaryMotor.config_kP(0, placementRotaryArmP);
    //rotaryMotor.config_kI(0, placementRotaryArmI);
    //rotaryMotor.config_kD(0, placementRotaryArmD);
    //rotaryMotor.config_kF(0, placementRotaryArmFUp);
    //rotaryMotor.configMotionSCurveStrength(0, sStrengthRotaryPlacementArm);
    rotaryMotor.configVoltageCompSaturation(Constants.kMaxRotaryArmVoltage);
    rotaryMotor.setNeutralMode(NeutralMode.Brake);
    rotaryMotor.enableVoltageCompensation(true);
    rotaryPID.setTolerance(Math.toRadians(2));

    rotaryMotor.setSensorPhase(true);

    Placement = Shuffleboard.getTab("rotary arm");
    n_placementRotaryArmP = Placement.add("p", placementRotaryArmP).getEntry();
    n_placementRotaryArmI = Placement.add("i", placementRotaryArmI).getEntry();
    n_placementRotaryArmD = Placement.add("d", placementRotaryArmD).getEntry();

    limitSwitch = new DigitalInput(Constants.ROTARY_ARM_LIMIT_SWITCH);

    // n_placementRotaryArmFUp = Placement.add("f up", placementRotaryArmFUp).getEntry();
    // n_placementRotaryArmFDown = Placement.add("f down", placementRotaryArmFDown).getEntry();
    // n_placementRotaryArmS_Strength = Placement.add("s strength", sStrengthRotaryPlacementArm).getEntry();

    positionMap = new HashMap<>();
    positionMap.put(Position.LOW, lowPosition);
    positionMap.put(Position.MID, midPosition);
    positionMap.put(Position.HIGH, highPosition);
    positionMap.put(Position.ZERO, zeroPosition);

    n_TestTab.add(rotaryPID);

    this.ligament = ligament;
  }

  /**
   * Reset the encoder position to 0
   */
  public void resetEncoder() {
    rotaryMotor.setSelectedSensorPosition(0);
  }

  /**
   * This method will be called once per scheduler run
   */
  @Override
  public void periodic() {
    ligament.setAngle(getAngleRotaryArm());
  }

  /**
   * Rotates the rotary arm
   * @param scalar value that scales the output speed from shuffleboard
   */
  public void rotateRotaryArm(double scalar){
    rotaryMotor.set(outputSpeed * scalar);
  }

  /**
   * This method will be called once per scheduler run during simulation
   */
  @Override
  public void simulationPeriodic() {}

  public boolean brokeLimit() {
    return !limitSwitch.get();
  }

  public void spin(double v) {
    rotaryMotor.set(ControlMode.PercentOutput, v);
  }

  /**
   * update the output speed, usually from network tables
   * @param newSpeed the new speed to set the output speed to
   */
  protected void updateOutputSpeed(double newSpeed) {
    outputSpeed = newSpeed;
  }

  /**
   * @return the output speed for the rotary arm
   */
  protected double getOutputSpeed() {
    return outputSpeed;
  }

  /**
   * Initializes the sendable builder to put on shuffleboard
   * @param builder sendable builder
   */
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Rotary arm");
    builder.addDoubleProperty("Rotary % output", this::getOutputSpeed, this::updateOutputSpeed);
    builder.addBooleanProperty("lim switch", this::brokeLimit, null);
    builder.addDoubleProperty("rotary length", this::getRawTicks, null);
    builder.addDoubleProperty("rotary angle", this::getPositionPlacementArmAngle, null);
    builder.addBooleanProperty("Brake", this::brakeIsEnabled, null);
  }

  /**
   * @return gets the angle of the rotary arm in raw ticks
   */
  public double getRawTicks() {
    return rotaryMotor.getSelectedSensorPosition();
  }

  /**
   * Stop the motor
   */
  public void stop() {
    rotaryMotor.set(0);
  }

  /**
   * @return Whether we are broke or not using default state
   */
  public boolean brakeIsEnabled() {
    return brake.get() ^ defaultState;
  }

  /**
   * Toggles the brake
   */
  public void toggleBrake(){
    brake.toggle();
  }

  /**
   * Engage brake using bitwise Xor default state
   */
  public void engageBrake() {
    // we shouldn't spin the motor when we engage the brake
    rotaryMotor.set(ControlMode.PercentOutput, 0);
    brake.set(true ^ defaultState);
  }

  /**
   * release the brake using bitwise Xor default state
   */
  public void releaseBrake(){
    brake.set(false ^ defaultState);
  }

  /**
   * Calculates the feed forward gain
   * @param extensionLength the length of the extension arm
   * @return the static feed forward gain
   */
  public double getFeedForward(double extensionLength){
    return Constants.kRotaryStaticGain * extensionLength * Math.sin(getPositionPlacementArmAngle());
  }

  /**
   * Go to position specified in the set point of the controller
   * @param extensionLength the length of the extension arm
   */
  public void gotoPos(double extensionLength) {
    double output = rotaryPID.calculate(getPositionPlacementArmAngle()) + getFeedForward(extensionLength);
    double setpoint = rotaryPID.getGoal().position;
    System.out.println("angle: " + getAngleRotaryArm());
    System.out.println("setpoint: " + setpoint);
    System.out.println("output: " + output);
/*    System.out.println("extension length: " + extensionLength);
    System.out.println("FF: " + getFeedForward((extensionLength)));*/
    rotaryMotor.set(ControlMode.PercentOutput, output);
  }

  /**
   * Make the setpoint for the controller low
   */
  public void makeSetpointLow(){
    rotaryPID.setGoal(lowPosition);
  }

  /**
   * make the setpoint for the controller mid
   */
  public void makeSetpointMid(){
    resetPIDController();
    rotaryPID.setGoal(midPosition);
  }

  public void resetPIDController() {
    rotaryPID.reset(getPositionPlacementArmAngle(), getSpeedPlacementArm());
  }

  /**
   * make the setpoint for the controller high
   */
  public void makeSetpointHigh(){
    resetPIDController();
    rotaryPID.setGoal(highPosition);
  }

  /**
   * make the setpoint for the controller 0
   */
  public void makeSetpointZero() {
    rotaryPID.setGoal(zeroPosition);
  }

  /**
   * @return position of the placement arm in radians
   */
  public double getPositionPlacementArmAngle(){
    return Constants.kTicksToRadiansRotaryPlacementArm * rotaryMotor.getSelectedSensorPosition();
  }

  /**
   * @return the speed of the placement arm in rads/sec
   */
  public double getSpeedPlacementArm(){
    return 10 * Constants.kTicksToRadiansRotaryPlacementArm * rotaryMotor.getSelectedSensorVelocity();
  }

  public void updateValues(){
    if (l_placementRotaryArmP != n_placementRotaryArmP.getDouble(placementRotaryArmP)){
      rotaryPID.setP(n_placementRotaryArmP.getDouble(placementRotaryArmP));
    }
    if (l_placementRotaryArmI != n_placementRotaryArmI.getDouble(placementRotaryArmI)){
      rotaryPID.setI(n_placementRotaryArmI.getDouble(placementRotaryArmI));
    }
    if (l_placementRotaryArmD != n_placementRotaryArmD.getDouble(placementRotaryArmD)){
      rotaryPID.setD(n_placementRotaryArmD.getDouble(placementRotaryArmD));
    }
/*
    if (l_placementRotaryArmFDown != n_placementRotaryArmFDown.getDouble(placementRotaryArmFDown)){
      rotaryMotor.config_kF(0, n_placementRotaryArmFDown.getDouble(placementRotaryArmFDown));
    }
    if (l_placementRotaryArmFUp != n_placementRotaryArmFUp.getDouble(placementRotaryArmFUp)){
      rotaryMotor.config_kF(0, n_placementRotaryArmFUp.getDouble(placementRotaryArmFUp));
    }
    if (l_placementRotaryArmS_Strength != n_placementRotaryArmS_Strength.getDouble(sStrengthRotaryPlacementArm)){
      rotaryMotor.configMotionSCurveStrength(0, (int) n_placementRotaryArmS_Strength.getDouble(sStrengthRotaryPlacementArm));
    }
    if (l_maxVelocityRotaryPlacementArm != n_maxVelocityRotaryPlacementArm.getDouble(Constants.kMaxVelocityRotaryPlacementArm)){
      rotaryMotor.configMotionCruiseVelocity( (int) n_maxVelocityRotaryPlacementArm.getDouble(Constants.kMaxVelocityRotaryPlacementArm),  0);
    }
    if (l_maxAccelerationRotaryPlacementArm != n_maxAccelerationRotaryPlacementArm.getDouble(Constants.kMaxAccelerationRotaryPlacementArm)){
      rotaryMotor.configMotionAcceleration((int) n_maxVelocityRotaryPlacementArm.getDouble(Constants.kMaxVelocityRotaryPlacementArm),  0);
    }
    */
  }

  public boolean isAtPosition() {
    return rotaryPID.atGoal();
  }

  public double getMidPosition(){
    return midPosition;
  }

  public double getHighPosition(){
    return highPosition;
  }

  public double getLowPosition(){
    return lowPosition;
  }

  public boolean goingUp() {
    return rotaryMotor.getSelectedSensorVelocity() > deadband;
  }

  public boolean isStationary() {
    return rotaryMotor.getSelectedSensorVelocity() < Math.abs(deadband);
  }

  public boolean goingDown() {
    return rotaryMotor.getSelectedSensorVelocity() < -deadband;
  }

  public double getStaticGain(double extensionArmLength) {
    return Math.sin(getPositionPlacementArmAngle()) * extensionArmLength * Constants.kRotaryStaticGain;
  }

  public boolean pastLimit() {
    return getPositionPlacementArmAngle() > Constants.kMaxRotaryLength;
  }

  public boolean outsideBumper() {
    return this.getPositionPlacementArmAngle() > Math.toRadians(30);
  }

  /**
   * @return gets the angle of the rotary arm
   */
  public double getAngleRotaryArm(){
    return Constants.kTicksToRadiansRotaryPlacementArm * rotaryMotor.getSelectedSensorPosition();
  }

}