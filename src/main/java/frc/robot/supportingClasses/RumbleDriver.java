package frc.robot.supportingClasses;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.RobotContainer.m_driverGamepad;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Newman_Constants.Constants;
import edu.wpi.first.wpilibj.TimedRobot;


public class RumbleDriver {
    private Timer timer1;

    public RumbleDriver () {
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