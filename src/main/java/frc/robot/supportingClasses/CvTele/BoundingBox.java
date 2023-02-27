package frc.robot.supportingClasses.CvTele;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Creates a 3D bounding box
 */
public class BoundingBox {
    // same coordinate system as Pose3Ds, mathematical grid
    protected double xSmall, ySmall, zSmall, xBig, yBig, zBig;

    /**
     * Creates a bounding box object
     *
     * @param xSmall the small x value in the bounding box
     * @param ySmall the small y value in the bounding box
     * @param zSmall the small z value in the bounding box
     * @param xBig the big x value in the bounding box
     * @param yBig the big y value in the bounding box
     * @param zBig the big z value in the bounding box
     */
    public BoundingBox(double xSmall, double ySmall, double zSmall, double xBig, double yBig, double zBig) {
        if (ySmall < 0 || yBig < 0) {
            DriverStation.reportError("Y value passed into bounding box is negative", false);
        }
        if (xSmall < xBig) {
            this.xSmall = xSmall;
            this.xBig = xBig;
        }
        else {
            this.xBig = xSmall;
            this.xSmall = xBig;
        }

        if (ySmall < yBig) {
            this.yBig = yBig;
            this.ySmall = ySmall;
        }
        else {
            this.ySmall = yBig;
            this.yBig = ySmall;
        }

        if (zSmall < zBig) {
            this.zBig = zBig;
            this.zSmall = zSmall;
        }
        else {
            this.zSmall = zBig;
            this.zBig = zSmall;
        }
    }

    /**
     *  A method that tells you if the passed in point is in the bounding box
     *
     * @param secondObjectPosition the position of the object you are comparing it to 3D space
     * @return the point is in the box
     */
    public boolean isInBox(Pose3d secondObjectPosition) {
        return this.isInBox(secondObjectPosition.toPose2d()) &&
                secondObjectPosition.getZ() < zBig && secondObjectPosition.getZ() > zSmall;
    }

    /**
     *  A method that tells you if the passed in point is in the bounding box
     *
     * @param secondObjectPosition the position of the object you are comparing it to 2D space
     * @return the point is in the box
     */
    public boolean isInBox(Pose2d secondObjectPosition) {
        return (secondObjectPosition.getX() < xBig && secondObjectPosition.getX() > xSmall) &&
                (secondObjectPosition.getY() < yBig && secondObjectPosition.getY() > ySmall);
    }
}
