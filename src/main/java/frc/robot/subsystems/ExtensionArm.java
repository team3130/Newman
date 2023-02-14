// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;
import frc.robot.supportingClasses.ShuffleboardUpdated;

public class ExtensionArm extends SubsystemBase implements ShuffleboardUpdated {
  public static double extensionArmSpeed = 0.6;
  public ShuffleboardTab tab = Shuffleboard.getTab("Test");
  public GenericEntry n_outputSpeed = tab.add("Extension % out", extensionArmSpeed).getEntry();
  /** Creates a new ExampleSubsystem. */
  public WPI_TalonSRX extensionMotor;
  public ExtensionArm() {
    extensionMotor = new WPI_TalonSRX(Constants.CAN_ExtensionArm);
    extensionMotor.configFactoryDefault();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * spins the extension arm
   * @param scalar to scale the output speed
   */
  public void spinExtensionArm(double scalar) {
    extensionMotor.set(scalar);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void updateValueFromShuffleboard() {
    extensionArmSpeed = n_outputSpeed.getDouble(extensionArmSpeed);
  }
}
