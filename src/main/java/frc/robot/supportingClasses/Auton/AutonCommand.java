package frc.robot.supportingClasses.Auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Stores the command to run as well as the start and end position of the bot as {@link Pose2d} objects.
 * objects of this class are frequently used for re-setting odometry to a position
 */
public class AutonCommand {
    protected Pose2d startPosition; // start position
    protected Pose2d endPosition; // end position
    protected HolonomicControllerCommand cmd; // command to run

    /**
     * Constructs an Auton Command object without the end position
     * @param cmd command to run
     * @param startPosition the start position of the trajectory
     */
    public AutonCommand(HolonomicControllerCommand cmd, Pose2d startPosition) {
        this.cmd = cmd;
        this.startPosition = startPosition;
    }

    /**
     * constructs the auton command
     * @param cmd command to run
     * @param startPosition the start position of the trajectory
     * @param endPosition the end position of the trajectory
     */
    public AutonCommand(HolonomicControllerCommand cmd, Pose2d startPosition, Pose2d endPosition) {
        this.cmd = cmd;
        this.startPosition = startPosition;
        this.endPosition = endPosition;
    }

    /**
     * @return the final position in the trajectory
     */
    public Pose2d getEndPosition() {
        return endPosition;
    }

    /**
     * Setter for the end position
     * @param endPosition the last position in the trajectory
     */
    public void setEndPosition(Pose2d endPosition) {
        this.endPosition = endPosition;
    }

    /**
     * sets the start position
     * @param newPosition position to set the start position to
     */
    public void setStartPosition(Pose2d newPosition) {
        startPosition = newPosition;
    }

    /**
     * sets the start position, sets the rotation to 0
     * @param x position (mathematical coordinates)
     * @param y position (mathematical coordinates)
     */
    public void setPosition(double x, double y) {
        setPosition(x, y, 0);
    }

    /**
     * sets the start position
     * @param x position (mathematical coordinates)
     * @param y position (mathematical coordinates)
     * @param rad rotation in radians (mathematical coordinates)
     */
    public void setPosition(double x, double y, double rad) {
        setPosition(x, y, new Rotation2d(rad));
    }

    /**
     * A setter for the start position and assigns it to the {@link Pose2d}
     * @param x position (mathematical coordinates)
     * @param y position (mathematical coordinates)
     * @param rotation rotation (mathematical coordinates)
     */
    public void setPosition(double x, double y, Rotation2d rotation) {
        startPosition = new Pose2d(x, y, rotation);
    }

    /**
     * @return the start position of the auton trajectory
     */
    public Pose2d getStartPosition() {
        return startPosition;
    }

    /**
     * @return the command to run
     */
    public CommandBase getCmd() {
        return cmd;
    }
}
