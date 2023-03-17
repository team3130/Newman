package frc.robot.supportingClasses.CvTele;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Hopper;
import frc.robot.supportingClasses.Auton.HolonomicControllerCommand;

import javax.swing.*;
import java.util.ArrayDeque;

/**
 * Action planner is a command that plans its next actions that it will take.
 * For example I'm running an auton path right now, I'll be here when it ends, and I need to do something when
 * I get to a certain point in the path
 */
public class ActionPlanner extends CommandBase {
    protected ArrayDeque<CommandBase> commandsToRun;
    protected Chassis m_chassis;
    protected Hopper m_hopper;
    protected

    public ActionPlanner(RobotContainer container) {

    }
}
