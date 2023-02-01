//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class ToggleIntake extends CommandBase {
    private final Intake m_intake;

    public ToggleIntake(Intake subsystem) {
        this.m_intake = subsystem;
    }

    public void initialize() {
        this.m_intake.setSpeed(0.5);
    }

    public void execute() {
        m_intake.ToggleLPneumatic();
        m_intake.ToggleSPneumatic();
        m_intake.setSpeed(-.5);
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {
        this.m_intake.setSpeed(0.0);
    }
}
