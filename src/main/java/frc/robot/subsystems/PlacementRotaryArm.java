// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;

import java.util.HashMap;

public class PlacementRotaryArm extends SubsystemBase {
  public enum Position {
    LOW, MID, HIGH
  }

  /** Creates a new ExampleSubsystem. */
  private WPI_TalonFX rotaryMotor;
  private double lowPosition = 0;
  private double midPosition = Math.PI/4;
  private double highPosition = Math.PI /2;

  private double positionDeadband = Math.toRadians(2.5);

  private long deadband = 100;

  private HashMap<Position, Double> positionMap;

  private GenericEntry n_lowPositionAngle;
  private GenericEntry n_midPositionAngle;
  private GenericEntry n_highPositionAngle;

  private double placementRotaryArmP = 5.12295e-5 / 2;
  private double placementRotaryArmI = 0;
  private double placementRotaryArmD = 0;
  private double placementRotaryArmFDown = 0;
  private double placementRotaryArmFUp = 0;
  private int sStrengthRotaryPlacementArm = 0;


  private ShuffleboardTab Placement;
  private GenericEntry n_placementRotaryArmP;
  private double l_placementRotaryArmP;
  private GenericEntry n_placementRotaryArmI;
  private double l_placementRotaryArmI;
  private GenericEntry n_placementRotaryArmD;
  private double l_placementRotaryArmD;
  private GenericEntry n_placementRotaryArmFUp;
  private double l_placementRotaryArmFUp;
  private GenericEntry n_placementRotaryArmFDown;
  private double l_placementRotaryArmFDown;
  private GenericEntry n_maxVelocityRotaryPlacementArm;
  private double l_maxVelocityRotaryPlacementArm;
  private GenericEntry n_maxAccelerationRotaryPlacementArm;
  private double l_maxAccelerationRotaryPlacementArm;
  private GenericEntry n_placementRotaryArmS_Strength;
  private double l_placementRotaryArmS_Strength;



  public PlacementRotaryArm() {
    rotaryMotor = new WPI_TalonFX(Constants.CAN_RotaryArm);
    rotaryMotor.configFactoryDefault();
    rotaryMotor.config_kP(0, placementRotaryArmP);
    rotaryMotor.config_kI(0, placementRotaryArmI);
    rotaryMotor.config_kI(0, placementRotaryArmD);
    rotaryMotor.config_kF(0, placementRotaryArmFUp);
    rotaryMotor.configMotionSCurveStrength(0, sStrengthRotaryPlacementArm);
    rotaryMotor.configVoltageCompSaturation(Constants.kMaxSteerVoltage);
    rotaryMotor.enableVoltageCompensation(true);

    Placement = Shuffleboard.getTab("rotary arm");
    n_placementRotaryArmP = Placement.add("p", placementRotaryArmP).getEntry();
    n_placementRotaryArmI = Placement.add("i", placementRotaryArmI).getEntry();
    n_placementRotaryArmD = Placement.add("d", placementRotaryArmD).getEntry();

    n_lowPositionAngle = Placement.add("low position", lowPosition).getEntry();
    n_midPositionAngle = Placement.add("mid position", midPosition).getEntry();
    n_highPositionAngle = Placement.add("high position", highPosition).getEntry();

    n_placementRotaryArmFUp = Placement.add("f up", placementRotaryArmFUp).getEntry();
    n_placementRotaryArmFDown = Placement.add("f down", placementRotaryArmFDown).getEntry();
    n_maxVelocityRotaryPlacementArm = Placement.add("max velocity", Constants.kMaxVelocityRotaryPlacementArm).getEntry();
    n_maxAccelerationRotaryPlacementArm = Placement.add("max acceleration", Constants.kMaxAccelerationRotaryPlacementArm).getEntry();
    n_placementRotaryArmS_Strength = Placement.add("s strength", sStrengthRotaryPlacementArm).getEntry();

    positionMap = new HashMap<>();
    positionMap.put(Position.LOW, lowPosition);
    positionMap.put(Position.MID, midPosition);
    positionMap.put(Position.HIGH, highPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void gotoLow(){
    rotaryMotor.set(ControlMode.MotionMagic, n_lowPositionAngle.getDouble(lowPosition), DemandType.ArbitraryFeedForward, n_placementRotaryArmFDown.getDouble(placementRotaryArmFDown));
  }
  public void goToMid(){
    if (getPositionPlacementArm() < midPosition) {
      rotaryMotor.set(ControlMode.MotionMagic, n_midPositionAngle.getDouble(midPosition), DemandType.ArbitraryFeedForward, n_placementRotaryArmFUp.getDouble(placementRotaryArmFUp));
    }
    else {
      rotaryMotor.set(ControlMode.MotionMagic, n_midPositionAngle.getDouble(midPosition), DemandType.ArbitraryFeedForward, n_placementRotaryArmFDown.getDouble(placementRotaryArmFDown));
    }
  }
  public void goToHigh(){
    rotaryMotor.set(ControlMode.MotionMagic, n_highPositionAngle.getDouble(highPosition), DemandType.ArbitraryFeedForward, n_placementRotaryArmFUp.getDouble(placementRotaryArmFUp));
  }

  public double getPositionPlacementArm(){
    return Constants.kTicksToRadiansRotaryPlacementArm * rotaryMotor.getSelectedSensorPosition();
  }
  public double getSpeedPlacementArm(){
    return 10 * Constants.kTicksToRadiansRotaryPlacementArm * rotaryMotor.getSelectedSensorVelocity();
  }
  public void updateValues(){
    if (l_placementRotaryArmP != n_placementRotaryArmP.getDouble(placementRotaryArmP)){
      rotaryMotor.config_kP(0, n_placementRotaryArmP.getDouble(placementRotaryArmP));
    }
    if (l_placementRotaryArmI != n_placementRotaryArmI.getDouble(placementRotaryArmI)){
      rotaryMotor.config_kI(0, n_placementRotaryArmI.getDouble(placementRotaryArmI));
    }
    if (l_placementRotaryArmD != n_placementRotaryArmD.getDouble(placementRotaryArmD)){
      rotaryMotor.config_kD(0, n_placementRotaryArmD.getDouble(placementRotaryArmD));
    }
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
  }

  public boolean isAtPosition(Position position) {
    return Math.abs(rotaryMotor.getSelectedSensorPosition() - positionMap.get(position)) < positionDeadband;
  }

  public void runAtPercentOutput(double output) {
    rotaryMotor.set(ControlMode.PercentOutput, output);
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

  public void stop() {
    rotaryMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * calculates the length of the placement spring
   * @param length extension arm length
   * @param angle rotary angle
   * @return A translation of the springs direction and magnitude
   */
  public Translation2d calculateExtensionArmSpringLength(double length, double angle) {
      double xSpring = length * Math.cos(angle);
      double ySpring = length * Math.sin(angle);

      return new Translation2d(
              Math.sqrt(Math.pow(xSpring - Constants.kExtensionArmSpringXPosition, 2) + Math.pow(ySpring - Constants.kExtensionArmSpringYPosition, 2)),
              new Rotation2d(Math.atan2(ySpring - Constants.kExtensionArmSpringYPosition, xSpring - Constants.kExtensionArmSpringXPosition))
      );
  }

  /**
  * Calculates the torque of the spring extension arm
  *
  * @param springStretch the result of {@link #calculateExtensionArmSpringLength} resembles the line that the spring
  *                      makes in the coordinate system: rotary arm axle is 0,0
  * @param rotaryAngle The angle of the rotary arm
  * @return the torque that the spring exerts on the rotary arm
  */
  public double calculateSpringExtensionArmTorque(Translation2d springStretch, double rotaryAngle) {
    // angle between where the force is pointing and the normal of the arm
     double thetaToNormal = springStretch.getAngle().getRadians() - rotaryAngle;
     // Hookes law
     double force = Constants.kExtensionArmSpringConstant * springStretch.getNorm();
     // torque formula: radius * force * sin(theta) where theta is the angle of the force to the normal of the arm
     return Constants.kExtensionArmSpringMountPosition * force * Math.sin(thetaToNormal);
  }

  /**
   * <p>
   *     Calculates the torque due to gravity that the axle currently experiences (without the spring).
   * </p>
   *
   * <p>
   * With standard feedforward you can have a second static gain called kG which is the gravity gain.
   * For us the amount of torque that the motor needs to produce isn't constant due to the torque changing.
   * Not only does the torque change because of the angle of the arm, the torque also changes due to the radius changing.
   * </p>
   *
   * @param rotaryAngle the angle of the rotary arm
   * @param extensionLength the length of the extension arm
   * @return the torque on the arm due to gravity
   */
  public double calculateTorqueDueToGravity(double rotaryAngle, double extensionLength) {
    // magic scalar of voltage to torque * force *
    double force = Constants.kAccelerationDueToGravity * Constants.kMassOfExtensionArm;
    // return the torque: radius * Force * sin(theta)
    return force * extensionLength * Math.sin(rotaryAngle);
  }

  /**
   * gets the net torque on the arm without a magic scalar
   * @param extensionArmLength the length of the extension arm
   * @return the net torque
   */
  public double getNetTorqueOnArm(double extensionArmLength) {
    double rotaryArm = getPositionPlacementArm();

    // the torque due to gravity - the upwards torque by the rotary arm
    return calculateTorqueDueToGravity(rotaryArm, extensionArmLength)
            -Math.abs(calculateSpringExtensionArmTorque(
                    calculateExtensionArmSpringLength(extensionArmLength, rotaryArm),
                    rotaryArm)
            );
  }

  public double getStaticGain(double extensionArmLength) {
    return getNetTorqueOnArm(extensionArmLength) * Constants.kTorqueToPercentOutScalar;
  }

}