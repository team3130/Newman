// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
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
    rotaryMotor.config_kF(0, placementRotaryArmFDown);
    rotaryMotor.configMotionSCurveStrength(0, sStrengthRotaryPlacementArm);

    Placement = Shuffleboard.getTab("rotary arm");
    n_placementRotaryArmP = Placement.add("p", placementRotaryArmP).getEntry();
    n_placementRotaryArmI = Placement.add("i", placementRotaryArmI).getEntry();
    n_placementRotaryArmD = Placement.add("d", placementRotaryArmD).getEntry();

    n_lowPositionAngle = Placement.add("low position", lowPosition).getEntry();
    n_midPositionAngle = Placement.add("mid position", midPosition).getEntry();
    n_highPositionAngle = Placement.add("high position", highPosition).getEntry();

    n_placementRotaryArmFUp = Placement.add("f up", placementRotaryArmFUp).getEntry();
    n_placementRotaryArmFDown = Placement.add("f down", placementRotaryArmFDown).getEntry();
    n_maxVelocityRotaryPlacementArm = Placement.add("max velocity", Constants.maxVelocityRotaryPlacementArm).getEntry();
    n_maxAccelerationRotaryPlacementArm = Placement.add("max acceleration", Constants.maxAccelerationRotaryPlacementArm).getEntry();
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
    return Constants.ticksToRadiansRotaryPlacementArm * rotaryMotor.getSelectedSensorPosition();
  }
  public double getSpeedPlacementArm(){
    return 10 * Constants.ticksToRadiansRotaryPlacementArm * rotaryMotor.getSelectedSensorVelocity();
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
    if (l_maxVelocityRotaryPlacementArm != n_maxVelocityRotaryPlacementArm.getDouble(Constants.maxVelocityRotaryPlacementArm)){
      rotaryMotor.configMotionCruiseVelocity( (int) n_maxVelocityRotaryPlacementArm.getDouble(Constants.maxVelocityRotaryPlacementArm),  0);
    }
    if (l_maxAccelerationRotaryPlacementArm != n_maxAccelerationRotaryPlacementArm.getDouble(Constants.maxAccelerationRotaryPlacementArm)){
      rotaryMotor.configMotionAcceleration((int) n_maxVelocityRotaryPlacementArm.getDouble(Constants.maxVelocityRotaryPlacementArm),  0);
    }
  }

  public boolean isAtPosition(Position position) {
    return Math.abs(rotaryMotor.getSelectedSensorPosition() - positionMap.get(position)) < positionDeadband;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
