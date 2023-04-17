package frc.robot.supportingClasses.Vision;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Odometry position. Holds both a position and a time that, that position was measured at.
 * The reason that we have this is because we want to update swerve drive odometry with a position 
 * and the time that we measured it for better accuracy. We also use this as a specific type for {@link frc.robot.supportingClasses.Vision.VisionMedianFilter}.
 */
public class OdoPosition {
    protected final Pose2d pose2d;
    protected final double timeStamp;

    /**
     * Position at a time
     */
    public OdoPosition(Pose2d pose2d, double timeStamp) {
        this.pose2d = pose2d;
        this.timeStamp = timeStamp;
    }

    /**
     * Getter for the position
     * @return the position measurement.
     */
    public Pose2d getPosition() {
        return pose2d;
    }

    /**
     * Getter for the time it occured at
     * @return the time that the measurement was taken at.
     */
    public double getTime() {
        return timeStamp;
    }

    /**
     * To string method for human readability of the object.
     * @return the string representation of the object.
     */
    public String toString() {
        return "position: " + pose2d.toString() + "time: " + timeStamp;
    }
}
