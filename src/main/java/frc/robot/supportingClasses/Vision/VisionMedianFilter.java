package frc.robot.supportingClasses.Vision;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Newman_Constants.Constants;

/**
 * A special type of median filter that is used to filter out outliers in vision data.
 */
public class VisionMedianFilter implements Sendable {
    // hehe
    protected int head;
    // size of elements we want in our moving window
    protected final int bucketSize;
    
    /**
     * Array for the x positional values
     */
    protected double[] x;
    protected double[] y;
    protected double[] yaw;
    protected double[] time;

    /**
     * Median filter for the x position
     */
    protected MedianFilter xFilter;

    /**
     * Median filter for the y position
     */
    protected MedianFilter yFilter;

    /**
     * Median filter for the yaw
     */
    protected MedianFilter yawFilter;

    /** Magic error scalar for radians to length */
    protected double kP;

    /**
     * Holds the amount of items we have added to the filter
     */
    protected int itemsAdded = 0;

    /**
     * The most recent pose that was added to the filter.
     */
    protected Pose2d mostRecentPose;

    /**
     * Constructs a VisionMedianFilter
     * @param bucketSize odd value for the bucket of the median filter
     *                   If the value is odd it will be truncated
     */
    public VisionMedianFilter(int bucketSize) {
        head = 0;
        if (bucketSize % 2 == 0) {
            bucketSize--;
        }

        else if (bucketSize < 1) {
            DriverStation.reportError("bucket size is too small", true);
        }

        x = new double[bucketSize];
        y = new double[bucketSize];
        yaw = new double[bucketSize];
        time = new double[bucketSize];

        xFilter = new MedianFilter(bucketSize);
        yFilter = new MedianFilter(bucketSize);
        yawFilter = new MedianFilter(bucketSize);

        this.bucketSize = bucketSize;

        kP = Constants.kKugelMedianFilterP;
    }

    public OdoPosition getOdoPose(OdoPosition position) {
        Pose2d pose = position.pose2d;
        x[head] = pose.getX();
        y[head] = pose.getY();
        yaw[head] = pose.getRotation().getRadians();
        time[head] = position.getTime();

        itemsAdded++;

        double medianX = xFilter.calculate(pose.getX());
        double medianY = yFilter.calculate(pose.getY());
        double medianYaw = yawFilter.calculate(pose.getRotation().getRadians());

        head = (head + 1) % bucketSize;

        // setup for the search
        double smallestErrorSeen = Double.MAX_VALUE;
        int indexOfSmallestError = bucketSize / 2;
        // find the smallest error
        for (int i = 0; i < bucketSize - 1; i++) {
            double error = Math.abs(x[i] - medianX) + Math.abs(y[i] - medianY) + kP * Math.abs(yaw[i] - medianYaw);
            if (error < smallestErrorSeen) {
                smallestErrorSeen = error;
                indexOfSmallestError = i;
            }
        }

        if (Constants.debugMode) {
            mostRecentPose = new Pose2d(x[indexOfSmallestError], y[indexOfSmallestError], new Rotation2d(yaw[indexOfSmallestError]));
        }

        // return the time and position of the best position estimate
        return new OdoPosition(
                new Pose2d(x[indexOfSmallestError], y[indexOfSmallestError], new Rotation2d(yaw[indexOfSmallestError])),
                time[indexOfSmallestError]);
    }

    public double getKp() {
        return kP;
    }

    public void setKp(double newKp) {
        kP = newKp;
    }

    public int getItemsAdded() {
        return itemsAdded;
    }

    public void setItemsAdded(long newItemsAdded) {
        itemsAdded = (int) newItemsAdded;
    }

    public int getHead() {
        return head;
    }

    public void setHead(long head) {
        this.head = (int) head;
    }

    public int getBucketSize() {
        return bucketSize;
    }

    public double[] getX() {
        return x;
    }

    public void setX(double[] x) {
        this.x = x;
    }

    public double[] getY() {
        return y;
    }

    public void setY(double[] y) {
        this.y = y;
    }

    public double[] getYaw() {
        return yaw;
    }

    public void setYaw(double[] yaw) {
        this.yaw = yaw;
    }

    public double[] getTime() {
        return time;
    }

    public void setTime(double[] time) {
        this.time = time;
    }

    public double getReadX() {
        return mostRecentPose.getX();
    }

    public double getReadY() {
        return mostRecentPose.getY();
    }

    public double getReadYaw() {
        return mostRecentPose.getRotation().getRadians();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Kugel Median Filter");
        builder.addDoubleProperty("Error scalar", this::getKp, this::setKp);
        builder.addIntegerProperty("Items added", this::getItemsAdded, this::setItemsAdded);
        builder.addDoubleArrayProperty("X values", this::getX, this::setX);
        builder.addDoubleArrayProperty("Y values", this::getY, this::setY);
        builder.addDoubleArrayProperty("Yaw values", this::getYaw, this::setYaw);
        builder.addDoubleArrayProperty("Time values", this::getTime, this::setTime);
        builder.addDoubleProperty("most recent read x", this::getReadX, null);
        builder.addDoubleProperty("most recent read y", this::getReadY, null);
        builder.addDoubleProperty("most recent read yaw", this::getReadYaw, null);
        builder.addIntegerProperty("head", this::getHead, this::setHead);
    }
}
