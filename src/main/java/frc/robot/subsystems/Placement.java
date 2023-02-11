// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenixpro.hardware.TalonFX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Placement extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public WPI_TalonFX rotaryArmMotor;
  public WPI_TalonFX extensionMotor;
  public Solenoid grabber;

  public double lowPosition = 0;
  public double midPosition = Math.PI/4;
  public double highPosition = Math.PI /2;

  public ShuffleboardTab Placement;
  public GenericEntry n_lowPositionAngle;
  public GenericEntry n_midPositionAngle;
  public GenericEntry n_highPositionAngle;
  public GenericEntry n_placementArmP;
  public double l_placementArmP;
  public GenericEntry n_placementArmI;
  public double l_placementArmI;
  public GenericEntry n_placementArmD;
  public double l_placementArmD;
  public GenericEntry n_placementArmFUp;
  public double l_placementArmFUp;
  public GenericEntry n_placementArmFDown;
  public double l_placementArmFDown;
  public GenericEntry maxVelocityPlacementArm;
  public GenericEntry maxAccelerationPlacementArm;
  public GenericEntry placementArmS_Strength;
  public double l_placementArmS_Strength;


  public Placement() {
    rotaryArmMotor = new WPI_TalonFX(Constants.CAN_RotaryArm);
    extensionMotor = new WPI_TalonFX(Constants.CAN_ExtensionMotor);
    rotaryArmMotor.configFactoryDefault();
    extensionMotor.configFactoryDefault();
    rotaryArmMotor.configMotionCruiseVelocity(Constants.maxVelocityPlacementArm);
    rotaryArmMotor.configMotionAcceleration(Constants.maxAccelerationPlacementArm);
    extensionMotor.config_kP(0,Constants.placementArmP);
    extensionMotor.config_kI(0,Constants.placementArmI);
    extensionMotor.config_kI(0,Constants.placementArmD);
    extensionMotor.config_kF(0,Constants.placementArmFUp);
    extensionMotor.config_kF(0,Constants.placementArmFDown);
    extensionMotor.configMotionSCurveStrength(0, Constants.sStrengthPlacementArm);
    grabber = new Solenoid(Constants.CAN_PNM, PneumaticsModuleType.CTREPCM,Constants.PNM_Grabber);

    Placement = Shuffleboard.getTab("placement");
    n_placementArmP = Placement.add("p", Constants.placementArmP).getEntry();
    n_placementArmI = Placement.add("i", Constants.placementArmI).getEntry();
    n_placementArmD = Placement.add("d", Constants.placementArmD).getEntry();
    n_lowPositionAngle = Placement.add("low position", lowPosition).getEntry();
    n_midPositionAngle = Placement.add("mid position", midPosition).getEntry();
    n_highPositionAngle = Placement.add("high position", highPosition).getEntry();
    n_placementArmFUp = Placement.add("f up", Constants.placementArmFUp).getEntry();
    n_placementArmFDown = Placement.add("f down", Constants.placementArmFDown).getEntry();
    maxVelocityPlacementArm = Placement.add("max velocity", Constants.maxVelocityPlacementArm).getEntry();
    maxAccelerationPlacementArm = Placement.add("max acceleration", Constants.maxAccelerationPlacementArm).getEntry();
    placementArmS_Strength = Placement.add("s strength", Constants.sStrengthPlacementArm).getEntry();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void RaiseRotaryArm(double speed){
    rotaryArmMotor.set(speed);
  }
  public void LowerRotaryArm(double speed){
    rotaryArmMotor.set(-speed);
  }
  public void ExtendExtensionArm(double speed){
    extensionMotor.set(speed);
  }
  public void RetractExtensionArm(double speed){
    extensionMotor.set(-speed);
  }


  public void gotoLow(){
    rotaryArmMotor.set(ControlMode.MotionMagic, n_lowPositionAngle.getDouble(lowPosition), DemandType.ArbitraryFeedForward, n_placementArmFDown.getDouble(Constants.placementArmFDown));
  }
  public void goToMid(){
    if (getPositionPlacementArm() < midPosition) {
      rotaryArmMotor.set(ControlMode.MotionMagic, n_midPositionAngle.getDouble(midPosition), DemandType.ArbitraryFeedForward, n_placementArmFUp.getDouble(Constants.placementArmFUp));
    }
    else {
      rotaryArmMotor.set(ControlMode.MotionMagic, n_midPositionAngle.getDouble(midPosition), DemandType.ArbitraryFeedForward, n_placementArmFDown.getDouble(Constants.placementArmFDown));
    }
  }
  public void goToHigh(){
    rotaryArmMotor.set(ControlMode.MotionMagic, n_highPositionAngle.getDouble(highPosition), DemandType.ArbitraryFeedForward, n_placementArmFUp.getDouble(Constants.placementArmFUp));
  }

  public double getPositionPlacementArm(){
    return Constants.ticksToRadiansPlacement * rotaryArmMotor.getSelectedSensorPosition();
  }
  public double getSpeedPlacementArm(){
    return 10 * Constants.ticksToRadiansPlacement * rotaryArmMotor.getSelectedSensorVelocity();
  }
  public void updateShuffleBoard(){
    if (l_placementArmP != n_placementArmP.getDouble(Constants.placementArmP)){
      rotaryArmMotor.config_kP(0, n_placementArmP.getDouble(Constants.placementArmP));
    }
    if (l_placementArmI != n_placementArmI.getDouble(Constants.placementArmI)){
      rotaryArmMotor.config_kI(0, n_placementArmI.getDouble(Constants.placementArmI));
    }
    if (l_placementArmD != n_placementArmD.getDouble(Constants.placementArmD)){
      rotaryArmMotor.config_kD(0, n_placementArmD.getDouble(Constants.placementArmD));
    }
    if (l_placementArmFDown != n_placementArmFDown.getDouble(Constants.placementArmFDown)){
      rotaryArmMotor.config_kF(0, n_placementArmFDown.getDouble(Constants.placementArmFDown));
    }
    if (l_placementArmFUp != n_placementArmFUp.getDouble(Constants.placementArmFUp)){
      rotaryArmMotor.config_kF(0, n_placementArmFUp.getDouble(Constants.placementArmFUp));
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
