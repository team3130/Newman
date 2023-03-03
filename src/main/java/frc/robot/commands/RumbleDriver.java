package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;



import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class RumbleDriver extends CommandBase {
    private Timer timer1;
    private final Joystick m_driverGamepad;

    public RumbleDriver (Joystick mDriverGamepad) {
        m_driverGamepad = mDriverGamepad;
        timer1 = new Timer();
        timer1.reset();
        timer1.start();

    }
    public void execute() {
            if (timer1.hasElapsed(120)) {
                m_driverGamepad.setRumble(GenericHID.RumbleType.kBothRumble, 1);
            }
            else {
                m_driverGamepad.setRumble(GenericHID.RumbleType.kBothRumble, 0);
            }

    }
}