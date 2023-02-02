//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

import static frc.robot.Constants.timetoIntake;

public class DeployIntake extends CommandBase {
    private final Intake m_intake;

    public DeployIntake(Intake subsystem) {
        this.m_intake = subsystem;
    }
    public static Timer timerintake = new Timer();
    public double Speed = -.5;
    public void initialize() {
        this.m_intake.setSpeed(Speed);
        m_intake.ToggleLPneumatic();
        timerintake.reset();
        timerintake.start();
    }
    public void execute() {
        if (timerintake.hasElapsed(timetoIntake)); {
            m_intake.ToggleSPneumatic();
        }
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {
        this.m_intake.setSpeed(0.0);
        m_intake.ToggleSPneumatic();
    }
}