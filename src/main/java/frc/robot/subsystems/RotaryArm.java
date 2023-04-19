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

/**
 * A subsystem that contains
 */
public class RotaryArm extends SubsystemBase {

  /**
   * The rotary arm motor controller
   */
  private final WPI_TalonFX rotaryMotor;

  /**
   * The solenoid that control the piston which enables the bike brake
   */
  private final Solenoid brake;

  /**
   * The default state of the brake
   */
  private final boolean defaultState = true;

  /**
   * A digital input on rio for the limit switch.
   */
  private final DigitalInput limitSwitch;

  /**
   * A generic entry for the brake indicator to be used during competition
   *    (which is why it isn't in the sendable as that is only put on shuffleboard in debug mode)
   */
  protected final GenericEntry n_brake;

  /**
   * The placement arm ligament in glass.
   */
  protected MechanismLigament2d ligament;

  /**
   * Trapezoid constraints outlined by {@link Constants#kMaxVelocityRotaryPlacementArm} for max velocity, and
   * {@link Constants#kMaxAccelerationRotaryPlacementArm} for max acceleration
   */
  private final TrapezoidProfile.Constraints rotaryArmConstraints;

  /**
   * the profiled pid controller for rotary arm. Follows a trajectory outlined by {@link #rotaryArmConstraints}
   */
  private final ProfiledPIDController rotaryPID;


  /**
   * Makes a new RotaryArm object.
   * @param ligament the ligament whose angle we update for the glass indicator
   */
  public RotaryArm(MechanismLigament2d ligament) {
    // rotary motor
    rotaryMotor = new WPI_TalonFX(Constants.CAN_RotaryArm);
    rotaryMotor.configFactoryDefault();
    rotaryMotor.configVoltageCompSaturation(Constants.kMaxRotaryArmVoltage);
    rotaryMotor.setNeutralMode(NeutralMode.Brake);
    rotaryMotor.enableVoltageCompensation(true);
    rotaryMotor.setSensorPhase(true); // the encoder's positive values are opposite the positive output of the motor

    // brake
    brake = new Solenoid(Constants.CAN_PNM, PneumaticsModuleType.CTREPCM, Constants.PNM_Brake);
    brake.set(defaultState);

    // constraints for pid
    rotaryArmConstraints = new TrapezoidProfile.Constraints(
            Constants.kMaxVelocityRotaryPlacementArm, Constants.kMaxAccelerationRotaryPlacementArm
    );

    // pid controller. Constructor is P, I, D, and trapezoid constraints
    rotaryPID = new ProfiledPIDController(
            Constants.kRotaryArmP, Constants.kRotaryArmI, Constants.kRotaryArmD, rotaryArmConstraints
    );
    rotaryPID.setTolerance(0.025);

    // limit switch to know where 0 is. Is located on the super structure of the bot.
    limitSwitch = new DigitalInput(Constants.ROTARY_ARM_LIMIT_SWITCH);

    this.ligament = ligament;

    // shuffleboard indicator for the brake
    n_brake = Shuffleboard.getTab("Comp").add("Brake indicator", false).getEntry();
  }

  /**
   * Reset the encoder position to 0
   */
  public void resetEncoder() {
    rotaryMotor.setSelectedSensorPosition(0);
  }

  /**
   * This method will be called once per scheduler run.
   * Updates the glass ligament.
   * Updates the brake boolean on shuffleboard.
   */
  @Override
  public void periodic() {
    ligament.setAngle(Math.toDegrees(getArmAngle()) - 90);
    n_brake.setBoolean(brakeIsEnabled());
  }

  /**
   * The boolean for the limit switch is inverted, so we must bitwise not the output of the {@link DigitalInput#get()} function.
   * @return whether the limit switch is broken on the rotary arm. (Are we at 0)
   */
  public boolean brokeLimit() {
    return !limitSwitch.get();
  }

  /**
   * Spins the motor at a percent output scaled by {@link Constants#kMaxRotaryArmVoltage}
   * @param output the speed to set the motor percent output at; values are between -1 - 1
   */
  public void spin(double output) {
    rotaryMotor.set(ControlMode.PercentOutput, output);
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
    // 24/34 is an approximate ratio between the weights on the extension arm and manipulator.
    return Constants.Extension.kRotaryStaticGain * (24d/34d * (extensionLength + Constants.Extension.kExtensionLengthRetracted)) * Math.sin(getArmAngle());
  }

  /**
   * Go to position specified in the set point of the controller
   * @param extensionLength the length of the extension arm
   */
  public void gotoPos(double extensionLength) {
    double output = rotaryPID.calculate(getArmAngle()) + getFeedForward(extensionLength);
    rotaryMotor.set(ControlMode.PercentOutput, output);
  }

  /**
   * Resets the PID controller and updates its setpoint with the passed in angle.
   * @param angle in radians to set the setpoint to of the PID controller
   */
  public void makeSetpoint(double angle) {
    resetPIDController();
    rotaryPID.setGoal(angle);
  }

  /**
   * Make the setpoint for the controller low
   */
  public void makeSetpointLow(){
    resetPIDController();
    rotaryPID.setGoal(Constants.lowPosition);
  }

  /**
   * Reset the PID controller to whatever angle we are currently reading and the speed we are currently reading.
   */
  public void resetPIDController() {
    rotaryPID.reset(getArmAngle(), getSpeedPlacementArm());
  }

  /**
   * @return position of the placement arm in radians
   */
  public double getArmAngle(){
    return Constants.kTicksToRadiansRotaryPlacementArm * rotaryMotor.getSelectedSensorPosition();
  }

  /**
   * @return the speed of the placement arm in rads/sec
   */
  public double getSpeedPlacementArm(){
    return 10 * Constants.kTicksToRadiansRotaryPlacementArm * rotaryMotor.getSelectedSensorVelocity();
  }

  /**
   * @return if the PID controller is at it's setpoint
   */
  public boolean isAtPosition() {
    return rotaryPID.atGoal();
  }

  /**
   * Current soft limit is 110 degrees.
   * TODO: move this value to Constants
   * @return if we are past the angle, soft limit on the rotary arm
   */
  public boolean pastLimit() {
    return getArmAngle() > Constants.highPosition;
  }

  /**
   * Current angle for the bumper is 30 degrees.
   * TODO: move this angle to Constants and find a better angle
   * @return If the extension arm is outside the bumper
   */
  public boolean outsideBumper() {
    return this.getArmAngle() > Math.toRadians(30);
  }

  /**
   * @param angle the angle in degrees to check the if the rotary arm is passed
   * @return whether the arm is past the passed in angle
   */
  public boolean pastAngle(double angle) {
    return this.getArmAngle() > Math.toRadians(angle);
  }

  /**
   * Initializes the sendable builder to put on shuffleboard
   * @param builder sendable builder
   */
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Rotary arm");
    builder.addBooleanProperty("lim switch", this::brokeLimit, null);
    builder.addDoubleProperty("rotary length", this::getRawTicks, null);
    builder.addDoubleProperty("rotary angle", this::getArmAngle, null);
    builder.addBooleanProperty("Brake", this::brakeIsEnabled, null);
  }

  /**
   * @return gets the angle of the rotary arm in raw ticks
   */
  public double getRawTicks() {
    return rotaryMotor.getSelectedSensorPosition();
  }

}