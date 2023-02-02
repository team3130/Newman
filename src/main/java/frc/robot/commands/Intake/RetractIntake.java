//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

import static frc.robot.Constants.timetoIntake;

public class RetractIntake extends CommandBase {
    private final Intake m_intake;

    private final Timer intakeTimer = new Timer();

    public RetractIntake(Intake subsystem) {
        this.m_intake = subsystem;
    }

    private boolean IntakeRan = false;
    public void initialize() {
        this.m_intake.setSpeed(0.0);
        intakeTimer.reset();
        intakeTimer.start();
    }


    public void execute() {
        if (intakeTimer.hasElapsed(timetoIntake)) {
            m_intake.ToggleLPneumatic();
            IntakeRan = true;
        }
    }
    public boolean isFinished () {
        return IntakeRan;
    }

    public void end(boolean interrupted) {
        IntakeRan = false;
    }
}
