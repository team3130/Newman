package frc.robot.supportingClasses.Auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PoseCommand extends CommandBase {
    protected Pose2d startPosition[]; // start position
    protected Pose2d endPosition[]; // end position
    protected final CommandBase[] cmd; // command to run
    protected final PathPlannerTrajectory trajectory[]; // the trajectory to follow
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
    }

    /**
     * Constructs a pose command without a trajectory or an end position
     * @param cmd the command tied into the PoseCommand object
     * @param startPosition the start position for this pose command
     */
    public PoseCommand(CommandBase cmd, Pose2d startPosition) {
        this.cmd = new CommandBase[] {cmd};
        this.startPosition = new Pose2d[] {startPosition};
        this.trajectory = null;
    }

    /**
     * Determine the command to run using your start position
     * @param currentPosition the position that the bot is currently at
     */
    public void determineCommand(Pose2d currentPosition) {
        if (Math.abs(currentPosition.getX()) <= 0.05 && Math.abs(currentPosition.getY()) <= 0.05) {
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
        }
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
