// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;
import frc.robot.supportingClasses.ShuffleboardUpdated;

public class RotaryArm extends SubsystemBase implements ShuffleboardUpdated {
  /** Creates a new ExampleSubsystem. */
  private double outputSpeed = 0.6;
  private ShuffleboardTab tab = Shuffleboard.getTab("Test");
  private GenericEntry n_outputSpeed = tab.add("Rotary arm % out", outputSpeed).getEntry();
  private WPI_TalonFX rotaryArmMotor;
  public RotaryArm() {
    rotaryArmMotor = new WPI_TalonFX(Constants.CAN_RotaryArm);
    rotaryArmMotor.configFactoryDefault();
    rotaryArmMotor.configVoltageCompSaturation(Constants.kMaxSteerVoltage);
    rotaryArmMotor.enableVoltageCompensation(true);
    rotaryArmMotor.setInverted(true);
    rotaryArmMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void updateValueFromShuffleboard() {
    outputSpeed = n_outputSpeed.getDouble(outputSpeed);
  }

  /**
   * Rotates the rotary arm
   * @param scalar value that scales the output speed from shuffleboard
   */
  public void rotateRotaryArm(double scalar){
    rotaryArmMotor.set(outputSpeed * scalar);
  }
  public double getAngleRotaryArm(){
    return Constants.ticksToRadiansRotaryPlacementArm * rotaryArmMotor.getSelectedSensorPosition();
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
