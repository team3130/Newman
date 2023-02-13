//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

import static frc.robot.Newman_Constants.Constants.timetoIntake;

public class DeployIntake extends CommandBase {
    private final Intake m_intake;

    public static Timer timerintake = new Timer();

    public DeployIntake(Intake subsystem) {
        this.m_intake = subsystem;
    }

    public void initialize() {
        m_intake.ToggleLPneumatic();
        timerintake.reset();
        timerintake.start();
    }

    public void execute() {
        if (timerintake.hasElapsed(timetoIntake)) {
            m_intake.ToggleSPneumatic();
            this.m_intake.setSpeed();
        }
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {
        this.m_intake.stop();
        m_intake.ToggleSPneumatic();
    }
}