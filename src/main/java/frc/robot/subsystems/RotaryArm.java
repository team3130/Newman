// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;

public class RotaryArm extends SubsystemBase {
  private final WPI_TalonFX rotaryMotor;
  private final Solenoid brake;
  private final boolean defaultState = true;

  private final DigitalInput limitSwitch;

  protected final GenericEntry n_brake;

  protected MechanismLigament2d ligament;
  private double outputSpeed = 0.6; // the speed we will run the rotary arm at

  public static final TrapezoidProfile.Constraints rotaryArmConstraints = new TrapezoidProfile.Constraints(
          Constants.kMaxVelocityRotaryPlacementArm, Constants.kMaxAccelerationRotaryPlacementArm);


  // the profiled pid controller for rotary arm
  public ProfiledPIDController rotaryPID = new ProfiledPIDController(Constants.kRotaryArmP, Constants.kRotaryArmI,
          Constants.kRotaryArmD, rotaryArmConstraints);


  public RotaryArm(MechanismLigament2d ligament) {
    rotaryMotor = new WPI_TalonFX(Constants.CAN_RotaryArm);
    rotaryMotor.configFactoryDefault();
    brake = new Solenoid(Constants.CAN_PNM, PneumaticsModuleType.CTREPCM, Constants.PNM_Brake);
    brake.set(defaultState);

    rotaryMotor.configVoltageCompSaturation(Constants.kMaxRotaryArmVoltage);
    rotaryMotor.setNeutralMode(NeutralMode.Brake);
    rotaryMotor.enableVoltageCompensation(true);
    rotaryPID.setTolerance(0.2);

    rotaryMotor.setSensorPhase(true);

    limitSwitch = new DigitalInput(Constants.ROTARY_ARM_LIMIT_SWITCH);

    this.ligament = ligament;

    n_brake = Shuffleboard.getTab("Comp").add("Brake indicator", false).getEntry();
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
    n_brake.setBoolean(brakeIsEnabled());
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
/*    System.out.println("extension length: " + extensionLength);
    System.out.println("FF: " + getFeedForward((extensionLength)));*/
    rotaryMotor.set(ControlMode.PercentOutput, output);
  }

  /**
   * Make the setpoint for the controller low
   */
  public void makeSetpointLow(){
    resetPIDController();
    rotaryPID.setGoal(Constants.lowPosition);
  }

  public void makeSetpointGroundCone() {
    resetPIDController();
    rotaryPID.setGoal(Constants.offGroundAngleCone);
  }

  /**
   * make the setpoint for the controller mid
   */
  public void makeSetpointMid(){
    resetPIDController();
    rotaryPID.setGoal(Constants.midPosition);
  }

  public void resetPIDController() {
    rotaryPID.reset(getPositionPlacementArmAngle(), getSpeedPlacementArm());
  }

  /**
   * make the setpoint for the controller high
   */
  public void makeSetpointHigh(){
    resetPIDController();
    rotaryPID.setGoal(Constants.highPosition);
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

  public boolean isAtPosition() {
    return rotaryPID.atGoal();
  }

  public double getStaticGain(double extensionArmLength) {
    return Math.sin(getPositionPlacementArmAngle()) * extensionArmLength * Constants.kRotaryStaticGain;
  }

  public boolean pastLimit() {
    return getPositionPlacementArmAngle() > Math.toRadians(110);
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