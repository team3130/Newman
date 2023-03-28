package frc.robot.supportingClasses;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.ExtensionArm;

/**
 * Creates a 3D bounding box
 */
public class BoundingBox {
    // same coordinate system as Pose3Ds, mathematical grid
    protected final double xSmall, ySmall, zSmall, xBig, yBig, zBig;

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
     * Wrapper for {@link #BoundingBox(double, double, double, double, double, double)}  BoundingBox},
     * initializes the small z value at -1 and the big z value to be 1000
     *
     * @param xSmall the small x value
     * @param ySmall the small y value
     * @param xBig the big x value
     * @param yBig the big y value
     */
    public BoundingBox(double xSmall, double ySmall, double xBig, double yBig) {
        this(xSmall, ySmall, -1, xBig, yBig, 1000);
    }

    /**
     *  Creates a bounding box from {@link Pose3d} objects.
     *
     * @param lowerCorner 3D point of the lower corner of the bounding box
     * @param upperCorner 3D point of the upper corner of the bounding box
     */
    public BoundingBox(Pose3d lowerCorner, Pose3d upperCorner) {
        this(lowerCorner.getX(), lowerCorner.getY(), lowerCorner.getZ(), upperCorner.getX(), upperCorner.getY(), upperCorner.getZ());
    }

    /**
     * Creates a bounding field from {@link Pose2d} objects
     *
     * @param lowerCorner 2D point of the lower corner of the bounding box.
     * @param upperCorner 2D point of the upper corner of the bounding box.
     */
    public BoundingBox(Pose2d lowerCorner, Pose2d upperCorner) {
        this(lowerCorner.getX(), lowerCorner.getY(), upperCorner.getX(), upperCorner.getY());
    }

    /**
     *  A method that tells you if the passed in point is in the bounding box
     *
     * @param secondObjectPosition the position of the object you are comparing it to 3D space
     * @return the point is in the box
     */
    /**
    public boolean isInBox(Pose3d secondObjectPosition) {
        return this.isInBox(secondObjectPosition.getX(), secondObjectPosition.getY(), secondObjectPosition.getZ());
    }
    */

    public boolean boxBad(Pose3d secondObjectPosition) {
        return this.isInBox2(secondObjectPosition.getX(), secondObjectPosition.getY(), secondObjectPosition.getZ());
    }

    /**
     * A method that tells you if the passed in point is in the bounding box
     *
     * @param x x coordinate of the point you want to check
     * @param y y coordinate of the point you want to check
     * @return whether the point is in the box or not
     */
    
    public boolean isInBox(double x, double y) {
        return (x > xSmall - 0.1 && x < xBig + 0.1) && (y > ySmall - 0.1 && y < yBig + 0.1);
    }
    
    /**
     * A method that tells you if the passed in point is in the bounding box
     *
     * @param x x coordinate of the point you want to check
     * @param y y coordinate of the point you want to check
     * @param z z coordinate of the point you want to check
     * @return whether the point is in the box
     * 
    */
    public boolean isInBox2(double x, double y, double z) {
        return this.isInBox(x, y) && (z > zSmall && z < zBig);
    }

    /**
     *  A method that tells you if the passed in point is in the bounding box.
     *
     * @param secondObjectPosition the position of the object you are comparing it to 2D space
     * @return the point is in the box
     */
    public boolean isInBox(Pose2d secondObjectPosition) {
        return this.isInBox(secondObjectPosition.getX(), secondObjectPosition.getY());
    }

    /**
     *  A method that tells you if the passed in bounding box has overlap with another bounding box.
     *
     * @param secondObjectPosition the other bounding box that you are comparing in space
     * @return the point is in the box
     */
    public boolean hasOverlapWith(BoundingBox secondObjectPosition) {
        return this.isInBox(secondObjectPosition.getLowPoint()) || this.isInBox(secondObjectPosition.getHighPoint());
    }

    /**
     * @return a Pose3D of the low point
     */
    public Pose3d getLowPoint() {
        return new Pose3d(xSmall, ySmall, zSmall, new Rotation3d());
    }

    /**
     * @return a Pose3D of the high point
     */
    public Pose3d getHighPoint() {
        return new Pose3d(xBig, yBig, zBig, new Rotation3d());
    }
}