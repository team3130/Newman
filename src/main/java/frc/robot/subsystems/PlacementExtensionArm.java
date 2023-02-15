// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;

public class PlacementExtensionArm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public WPI_TalonFX extensionMotor;


  //general
  public ShuffleboardTab Placement;
  public GenericEntry n_placementExtensionArmP;
  public double l_placementExtensionArmP;
  public GenericEntry n_placementExtensionArmI;
  public double l_placementExtensionArmI;
  public GenericEntry n_placementExtensionArmD;
  public double l_placementExtensionArmD;
  public GenericEntry n_placementExtensionArmFUp;
  public double l_placementExtensionArmFUp;
  public GenericEntry n_placementExtensionArmFDown;
  public double l_placementExtensionArmFDown;
  public GenericEntry n_maxVelocityPlacementExtensionArm;
  public double l_maxVelocityPlacementExtensionArm;
  public GenericEntry n_maxAccelerationPlacementExtensionArm;
  public double l_maxAccelerationPlacementExtensionArm;
  public GenericEntry n_placementExtensionArmS_Strength;
  public double l_placementExtensionArmS_Strength;

  public GenericEntry n_collapsedPosition;
  public GenericEntry n_intermediatePosition;
  public GenericEntry n_extendedPosition;
  public double collapsedPosition = 0;
  public double intermediatePosition = 1;
  public double extendedPosition = 2;

  public double placementExtensionArmP = 5.12295e-5 / 2;
  public double placementExtensionArmI = 0;
  public double placementExtensionArmD = 0;
  public double placementExtensionArmFDown = 0;
  public double placementExtensionArmFUp = 0;
  public int sStrengthPlacementExtensionArm = 0;



  public PlacementExtensionArm() {
    extensionMotor = new WPI_TalonFX(Constants.CAN_ExtensionArm);
    extensionMotor.configFactoryDefault();
    extensionMotor.config_kP(0,placementExtensionArmP);
    extensionMotor.config_kI(0,placementExtensionArmI);
    extensionMotor.config_kI(0,placementExtensionArmD);
    extensionMotor.config_kF(0,placementExtensionArmFUp);
    extensionMotor.config_kF(0,placementExtensionArmFDown);
    extensionMotor.configMotionSCurveStrength(0, sStrengthPlacementExtensionArm);
    extensionMotor.configVoltageCompSaturation(Constants.kMaxSteerVoltage);
    extensionMotor.enableVoltageCompensation(true);

    Placement = Shuffleboard.getTab("Extension Arm");
    n_placementExtensionArmP = Placement.add("p", placementExtensionArmP).getEntry();
    n_placementExtensionArmI = Placement.add("i", placementExtensionArmI).getEntry();
    n_placementExtensionArmD = Placement.add("d", placementExtensionArmD).getEntry();


    n_collapsedPosition = Placement.add("collapses position", collapsedPosition).getEntry();
    n_intermediatePosition = Placement.add("intermediate positon", intermediatePosition).getEntry();
    n_extendedPosition = Placement.add("extended position", extendedPosition).getEntry();


    n_placementExtensionArmFUp = Placement.add("f up", placementExtensionArmFUp).getEntry();
    n_placementExtensionArmFDown = Placement.add("f down", placementExtensionArmFDown).getEntry();
    n_maxVelocityPlacementExtensionArm = Placement.add("max velocity", Constants.maxVelocityPlacementExtensionArm).getEntry();
    n_maxAccelerationPlacementExtensionArm = Placement.add("max acceleration", Constants.maxAccelerationPlacementExtensionArm).getEntry();
    n_placementExtensionArmS_Strength = Placement.add("s strength", sStrengthPlacementExtensionArm).getEntry();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void extendArm(){
    extensionMotor.set(ControlMode.MotionMagic, n_collapsedPosition.getDouble(n_collapsedPosition.getDouble(collapsedPosition)));
  }
  public void intermediateArm(){
    extensionMotor.set(ControlMode.MotionMagic, n_intermediatePosition.getDouble(n_intermediatePosition.getDouble(intermediatePosition)));
  }
  public void collapseArm(){
    extensionMotor.set(ControlMode.MotionMagic, n_collapsedPosition.getDouble(n_collapsedPosition.getDouble(collapsedPosition)));
  }




  public double getPositionPlacementArm(){
    return Constants.ticksToRadiansExtensionPlacement * extensionMotor.getSelectedSensorPosition();
  }
  public double getSpeedPlacementArm(){
    return 10 * Constants.ticksToRadiansExtensionPlacement * extensionMotor.getSelectedSensorVelocity();
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
    if (l_placementExtensionArmFDown != n_placementExtensionArmFDown.getDouble(placementExtensionArmFDown)){
      extensionMotor.config_kF(0, n_placementExtensionArmFDown.getDouble(placementExtensionArmFDown));
    }
    if (l_placementExtensionArmFUp != n_placementExtensionArmFUp.getDouble(placementExtensionArmFUp)){
      extensionMotor.config_kF(0, n_placementExtensionArmFUp.getDouble(placementExtensionArmFUp));
    }
    if (l_placementExtensionArmS_Strength != n_placementExtensionArmS_Strength.getDouble(sStrengthPlacementExtensionArm)){
      extensionMotor.configMotionSCurveStrength(0, (int) n_placementExtensionArmS_Strength.getDouble(sStrengthPlacementExtensionArm));
    }
    if (l_maxVelocityPlacementExtensionArm != n_maxVelocityPlacementExtensionArm.getDouble(Constants.maxVelocityPlacementExtensionArm)){
      extensionMotor.configMotionCruiseVelocity( (int) n_maxVelocityPlacementExtensionArm.getDouble(Constants.maxVelocityPlacementExtensionArm),  0);
    }
    if (l_maxAccelerationPlacementExtensionArm != n_maxAccelerationPlacementExtensionArm.getDouble(Constants.maxAccelerationPlacementExtensionArm)){
      extensionMotor.configMotionAcceleration((int) n_maxVelocityPlacementExtensionArm.getDouble(Constants.maxVelocityPlacementExtensionArm),  0);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
