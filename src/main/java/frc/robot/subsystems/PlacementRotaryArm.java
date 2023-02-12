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
import frc.robot.Constants.Constants;

public class PlacementRotaryArm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public WPI_TalonFX rotaryMotor;
  public double lowPosition = 0;
  public double midPosition = Math.PI/4;
  public double highPosition = Math.PI /2;

  public GenericEntry n_lowPositionAngle;
  public GenericEntry n_midPositionAngle;
  public GenericEntry n_highPositionAngle;

  public double placementRotaryArmP = 5.12295e-5 / 2;
  public double placementRotaryArmI = 0;
  public double placementRotaryArmD = 0;
  public double placementRotaryArmFDown = 0;
  public double placementRotaryArmFUp = 0;
  public int sStrengthRotaryPlacementArm = 0;


  public ShuffleboardTab Placement;
  public GenericEntry n_placementRotaryArmP;
  public double l_placementRotaryArmP;
  public GenericEntry n_placementRotaryArmI;
  public double l_placementRotaryArmI;
  public GenericEntry n_placementRotaryArmD;
  public double l_placementRotaryArmD;
  public GenericEntry n_placementRotaryArmFUp;
  public double l_placementRotaryArmFUp;
  public GenericEntry n_placementRotaryArmFDown;
  public double l_placementRotaryArmFDown;
  public GenericEntry n_maxVelocityRotaryPlacementArm;
  public double l_maxVelocityRotaryPlacementArm;
  public GenericEntry n_maxAccelerationRotaryPlacementArm;
  public double l_maxAccelerationRotaryPlacementArm;
  public GenericEntry n_placementRotaryArmS_Strength;
  public double l_placementRotaryArmS_Strength;



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

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
