package frc.robot.supportingClasses.Auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PoseCommand extends CommandBase {
    protected Pose2d startPosition; // start position
    protected Pose2d endPosition; // end position
    protected final CommandBase cmd; // command to run
    protected final PathPlannerTrajectory trajectory; // the trajectory to follow

    /**
     * Creates a new pose command using the trajectory and holonomic controller
     * @param cmd the command that is tied into the pose command object
     * @param trajectory the trajectory that is followed in the pose command
     */
    public PoseCommand(CommandBase cmd, PathPlannerTrajectory trajectory) {
        startPosition = trajectory.getInitialPose();
        endPosition = trajectory.getEndState().poseMeters;
        this.cmd = cmd;
        this.trajectory = trajectory;

        m_requirements.addAll(cmd.getRequirements());
    }

    /**
     * Constructs a pose command without a trajectory or an end position
     * @param cmd the command tied into the PoseCommand object
     * @param startPosition the start position for this pose command
     */
    public PoseCommand(CommandBase cmd, Pose2d startPosition) {
        this.cmd = cmd;
        this.startPosition = startPosition;
        this.trajectory = null;
    }

    @Override
    public void initialize() {
        cmd.initialize();
    }

    @Override
    public void execute() {
        cmd.execute();
    }

    @Override
    public boolean isFinished() {
        return cmd.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        cmd.end(interrupted);
    }

    /**
     * @return The start position of the command that is running
     */
    public Pose2d getStartPosition() {
        return startPosition;
    }

    /**
     * @return The end position of the command that is running
     */
    public Pose2d getEndPosition() {
        return endPosition;
    }

    /**
     * @return the start holonomic rotation of the path that will be ran
     */
    public Rotation2d getStartRotation() {
        return trajectory.getInitialState().holonomicRotation;
    }

    /**
     * @return the trajectory we are running
     */
    public PathPlannerTrajectory getTrajectory() {
        return trajectory;
    }
}
