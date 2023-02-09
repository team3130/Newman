// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
public class Hopper extends SubsystemBase {
    protected final WPI_TalonSRX leftwheel;
    protected final WPI_TalonSRX rightwheel;
    private static ShuffleboardTab TAB = Shuffleboard.getTab("Hopper");
    public Hopper() {
        leftwheel = new WPI_TalonSRX(Constants.CAN_hopperleft);
        rightwheel = new WPI_TalonSRX(Constants.CAN_hopperright);
        leftwheel.configFactoryDefault();
        rightwheel.configFactoryDefault();
    }
public void spinMotor (double output) {
        leftwheel.set(ControlMode.PercentOutput, output);
        rightwheel.set(ControlMode.PercentOutput, output);
}
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
//sus