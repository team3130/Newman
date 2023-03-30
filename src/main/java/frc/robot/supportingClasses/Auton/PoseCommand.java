package frc.robot.supportingClasses.Auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.Arrays;

public class PoseCommand extends CommandBase {
    protected Pose2d[] startPosition; // start position
    protected Pose2d[] endPosition; // end position
    protected final CommandBase[] cmd; // command to run
    protected final PathPlannerTrajectory[] trajectory; // the trajectory to follow
    protected int index = 0; // the index of cmd to run. used in human player-station side logic

    /**
     * Creates a new pose command using the trajectory and holonomic controller
     * @param cmd the command that is tied into the pose command object
     * @param trajectory the trajectory that is followed in the pose command
     */
    public PoseCommand(CommandBase cmd, PathPlannerTrajectory trajectory) {
        startPosition = new Pose2d[] {trajectory.getInitialPose()};
        endPosition = new Pose2d[] {trajectory.getEndState().poseMeters};
        this.cmd = new CommandBase[] {cmd};
        this.trajectory = new PathPlannerTrajectory[] {trajectory};

        m_requirements.addAll(cmd.getRequirements());
        CommandScheduler.getInstance().registerComposedCommands(cmd);
    }

    public PoseCommand(AutonCommand command) {
        this(command, command.trajectory);
    }

    /**
     * Constructs a pose command without a trajectory or an end position
     * @param cmd the command tied into the PoseCommand object
     * @param startPosition the start position for this pose command
     */
    public PoseCommand(CommandBase cmd, Pose2d startPosition) {
        this.cmd = new CommandBase[] {cmd};
        this.trajectory = null;
        this.startPosition = new Pose2d[] {startPosition};
        m_requirements.addAll(cmd.getRequirements());
        CommandScheduler.getInstance().registerComposedCommands(cmd);

    }

    public PoseCommand(CommandBase cmd1, Pose2d startPosition1, CommandBase cmd2, Pose2d startPosition2) {
        this.cmd = new CommandBase[] {cmd1, cmd2};
        this.startPosition = new Pose2d[] {startPosition1, startPosition2};
        this.trajectory = null;
        m_requirements.addAll(cmd1.getRequirements());
        m_requirements.addAll(cmd2.getRequirements());
        CommandScheduler.getInstance().registerComposedCommands(cmd1, cmd2);

    }

    /**
     * Makes a new PoseCommand
     * @param cmd an array of the commands being followed
     * @param trajectory an array of the trajectories being followed
     */
    public PoseCommand(CommandBase[] cmd, PathPlannerTrajectory[] trajectory) {
        this.cmd = cmd;
        this.trajectory = trajectory;
        startPosition = new Pose2d[trajectory.length];
        endPosition = new Pose2d[trajectory.length];
        for (int i = 0; i < startPosition.length; i++) {
            startPosition[i] = trajectory[i].getInitialPose();
            endPosition[i] = trajectory[i].getEndState().poseMeters;
            m_requirements.addAll(cmd[i].getRequirements());
        }
        CommandScheduler.getInstance().registerComposedCommands(cmd);
    }

    /**
     * Makes a new PoseCommand object using the command and trajectories passed in, in order.
     * @param cmd1 the first command
     * @param trajectory1 the trajectory one
     * @param cmd2 the second command
     * @param trajectory2 the second trajectory
     */
    public PoseCommand(CommandBase cmd1, PathPlannerTrajectory trajectory1, CommandBase cmd2, PathPlannerTrajectory trajectory2) {
        this(new CommandBase[] {cmd1, cmd2}, new PathPlannerTrajectory[] {trajectory1, trajectory2});
    }

    /**
     * Determine the command to run using your start position
     * @param currentPosition the position that the bot is currently at
     */
    public void determineCommand(Pose2d currentPosition) {
        index = 0;
/*        if (Math.abs(currentPosition.getX()) <= 0.05 && Math.abs(currentPosition.getY()) <= 0.05) {
            index = 0;
        }
        else {
            double lowest = Double.POSITIVE_INFINITY;
            int indexOfLowest = 0;
            for (int i = 0; i < cmd.length; i++) {
                double curr = currentPosition.relativeTo(startPosition[i]).getTranslation().getNorm();
                if (lowest > curr) {
                    lowest = curr;
                    indexOfLowest = i;
                }
            }
            index = indexOfLowest;
        }*/
    }

    public PoseCommand(AutonCommand command1, AutonCommand command2) {
        this(command1, command1.getTrajectory(), command2, command2.getTrajectory());
    }

    @Override
    public void initialize() {
        cmd[index].initialize();
    }

    @Override
    public void execute() {
        cmd[index].execute();
    }

    @Override
    public boolean isFinished() {
        return cmd[index].isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        cmd[index].end(interrupted);
    }

    /**
     * @return The start position of the command that is running
     */
    public Pose2d getStartPosition() {
        return startPosition[index];
    }

    /**
     * @return The end position of the command that is running
     */
    public Pose2d getEndPosition() {
        return endPosition[index];
    }

    /**
     * @return the start holonomic rotation of the path that will be ran
     */
    public Rotation2d getStartRotation() {
        return trajectory[index].getInitialState().holonomicRotation;
    }

    /**
     * @return the trajectory we are running
     */
    public PathPlannerTrajectory getTrajectory() {
        return trajectory[index];
    }
}
