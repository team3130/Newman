//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class ToggleIntake extends CommandBase {
    private final Intake m_intake;

    public ToggleIntake(Intake subsystem) {
        this.m_intake = subsystem;
    }
    double Speed = -.5;
    public void initialize() {
        this.m_intake.setSpeed(Speed);
        m_intake.ToggleLPneumatic();
        timerintake.reset();
        timerintake.start();
    }
    private final Timer timerintake = new Timer();
    private final double timetoIntake = 0.25;

    public void execute() {
        if (timerintake.hasElapsed(0.25)); {
            m_intake.ToggleSPneumatic();
            m_intake.setSpeed(Speed);
        }
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {
        this.m_intake.setSpeed(0.0);
    }
}
