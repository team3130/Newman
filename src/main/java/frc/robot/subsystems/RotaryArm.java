// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;

/**
 * The rotary arm
 */
public class RotaryArm extends SubsystemBase {
  /**
   * mechanism 2d to show the extension arm length
   */
  protected MechanismLigament2d ligament;

  private double outputSpeed = 0.6; // the speed we will run the rotary arm at

  private final WPI_TalonFX rotaryArmMotor; // motor for the rotary arm

  /**
   * Constructs a rotary arm in brake mode with 9 volts, voltage compensation
   *
   * @param ligament the ligament object that is on smart-dashboard
   */

  public RotaryArm(MechanismLigament2d ligament) {
    rotaryArmMotor = new WPI_TalonFX(Constants.CAN_RotaryArm);
    rotaryArmMotor.configFactoryDefault();
    rotaryArmMotor.configVoltageCompSaturation(Constants.kMaxSteerVoltage);
    rotaryArmMotor.enableVoltageCompensation(true);
    rotaryArmMotor.setInverted(true);
    rotaryArmMotor.setNeutralMode(NeutralMode.Brake);

    this.ligament = ligament;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // ligament.setAngle() TODO: use this for drawing it
  }

  /**
   * Rotates the rotary arm
   * @param scalar value that scales the output speed from shuffleboard
   */
  public void rotateRotaryArm(double scalar){
    rotaryArmMotor.set(outputSpeed * scalar);
  }

  /**
   * @return gets the angle of the rotary arm
   */
  public double getAngleRotaryArm(){
    return Constants.kTicksToRadiansRotaryPlacementArm * rotaryArmMotor.getSelectedSensorPosition();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
