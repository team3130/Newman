package frc.robot.supportingClasses;

import edu.wpi.first.math.geometry.Pose2d;

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

    public Pose2d getPosition() {
        return pose2d;
    }

    public double getTime() {
        return timeStamp;
    }

    public String toString() {
        return "position: " + pose2d.toString() + "time: " + timeStamp;
    }
}
