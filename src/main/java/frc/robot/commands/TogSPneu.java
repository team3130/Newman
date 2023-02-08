//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
public class TogSPneu extends CommandBase {
    private final Intake m_intake;
    public TogSPneu(Intake subsystem) {
        this.m_intake = subsystem;
    }
    public void initialize (){
        m_intake.ToggleSPneumatic();
        this.m_intake.setSpeed(0.0);
    }
    public void execute(){

    }
}
