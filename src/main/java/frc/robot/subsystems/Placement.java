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
import frc.robot.Constants.RangeBasedMotor;

public class Placement extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public WPI_TalonFX rangeBasedMotor;

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
  public GenericEntry n_maxVelocityPlacementArm;
  public double l_maxVelocityPlacementArm;
  public GenericEntry n_maxAccelerationPlacementArm;
  public double l_maxAccelerationPlacementArm;
  public GenericEntry n_placementArmS_Strength;
  public double l_placementArmS_Strength;

  public RangeBasedMotor constants;


  public Placement(RangeBasedMotor constants) {
    rangeBasedMotor = new WPI_TalonFX(constants.CAN_ID);
    rangeBasedMotor.configFactoryDefault();
    rangeBasedMotor.config_kP(0,constants.placementArmP);
    rangeBasedMotor.config_kI(0,constants.placementArmI);
    rangeBasedMotor.config_kI(0,constants.placementArmD);
    rangeBasedMotor.config_kF(0,constants.placementArmFUp);
    rangeBasedMotor.config_kF(0,constants.placementArmFDown);
    rangeBasedMotor.configMotionSCurveStrength(0, constants.sStrengthPlacementArm);

    Placement = Shuffleboard.getTab("placement");
    n_placementArmP = Placement.add("p", constants.placementArmP).getEntry();
    n_placementArmI = Placement.add("i", constants.placementArmI).getEntry();
    n_placementArmD = Placement.add("d", constants.placementArmD).getEntry();
    n_lowPositionAngle = Placement.add("low position", lowPosition).getEntry();
    n_midPositionAngle = Placement.add("mid position", midPosition).getEntry();
    n_highPositionAngle = Placement.add("high position", highPosition).getEntry();
    n_placementArmFUp = Placement.add("f up", constants.placementArmFUp).getEntry();
    n_placementArmFDown = Placement.add("f down", constants.placementArmFDown).getEntry();
    n_maxVelocityPlacementArm = Placement.add("max velocity", constants.maxVelocityPlacementArm).getEntry();
    n_maxAccelerationPlacementArm = Placement.add("max acceleration", constants.maxAccelerationPlacementArm).getEntry();
    n_placementArmS_Strength = Placement.add("s strength", constants.sStrengthPlacementArm).getEntry();

    this.constants = constants;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void ExtendExtensionArm(double speed){
    rangeBasedMotor.set(speed);
  }
  public void RetractExtensionArm(double speed){
    rangeBasedMotor.set(-speed);
  }


  public void gotoLow(){
    rangeBasedMotor.set(ControlMode.MotionMagic, n_lowPositionAngle.getDouble(lowPosition), DemandType.ArbitraryFeedForward, n_placementArmFDown.getDouble(constants.placementArmFDown));
  }
  public void goToMid(){
    if (getPositionPlacementArm() < midPosition) {
      rangeBasedMotor.set(ControlMode.MotionMagic, n_midPositionAngle.getDouble(midPosition), DemandType.ArbitraryFeedForward, n_placementArmFUp.getDouble(constants.placementArmFUp));
    }
    else {
      rangeBasedMotor.set(ControlMode.MotionMagic, n_midPositionAngle.getDouble(midPosition), DemandType.ArbitraryFeedForward, n_placementArmFDown.getDouble(constants.placementArmFDown));
    }
  }
  public void goToHigh(){
    rangeBasedMotor.set(ControlMode.MotionMagic, n_highPositionAngle.getDouble(highPosition), DemandType.ArbitraryFeedForward, n_placementArmFUp.getDouble(constants.placementArmFUp));
  }

  public double getPositionPlacementArm(){
    return constants.ticksToRadiansPlacement * rangeBasedMotor.getSelectedSensorPosition();
  }
  public double getSpeedPlacementArm(){
    return 10 * constants.ticksToRadiansPlacement * rangeBasedMotor.getSelectedSensorVelocity();
  }
  public void updateValues(){
    if (l_placementArmP != n_placementArmP.getDouble(constants.placementArmP)){
      rangeBasedMotor.config_kP(0, n_placementArmP.getDouble(constants.placementArmP));
    }
    if (l_placementArmI != n_placementArmI.getDouble(constants.placementArmI)){
      rangeBasedMotor.config_kI(0, n_placementArmI.getDouble(constants.placementArmI));
    }
    if (l_placementArmD != n_placementArmD.getDouble(constants.placementArmD)){
      rangeBasedMotor.config_kD(0, n_placementArmD.getDouble(constants.placementArmD));
    }
    if (l_placementArmFDown != n_placementArmFDown.getDouble(constants.placementArmFDown)){
      rangeBasedMotor.config_kF(0, n_placementArmFDown.getDouble(constants.placementArmFDown));
    }
    if (l_placementArmFUp != n_placementArmFUp.getDouble(constants.placementArmFUp)){
      rangeBasedMotor.config_kF(0, n_placementArmFUp.getDouble(constants.placementArmFUp));
    }
    if (l_placementArmS_Strength != n_placementArmS_Strength.getDouble(constants.sStrengthPlacementArm)){
      rangeBasedMotor.configMotionSCurveStrength(0, (int) n_placementArmS_Strength.getDouble(constants.sStrengthPlacementArm));
    }
    if (l_maxVelocityPlacementArm != n_maxVelocityPlacementArm.getDouble(constants.maxVelocityPlacementArm)){
      rangeBasedMotor.configMotionCruiseVelocity( (int) n_maxVelocityPlacementArm.getDouble(constants.maxVelocityPlacementArm),  0);
    }
    if (l_maxAccelerationPlacementArm != n_maxAccelerationPlacementArm.getDouble(constants.maxAccelerationPlacementArm)){
      rangeBasedMotor.configMotionAcceleration((int) n_maxVelocityPlacementArm.getDouble(constants.maxVelocityPlacementArm),  0);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
