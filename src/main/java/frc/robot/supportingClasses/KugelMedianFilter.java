package frc.robot.supportingClasses;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class KugelMedianFilter {
    // hehe
    protected int head;
    // size of elements we want in our moving window
    protected final int bucketSize;
    // raw values
    protected double[] x;
    protected double[] y;
    protected double[] yaw;
    protected double[] time;

    // heaps
    protected MedianFilter xFilter;
    protected MedianFilter yFilter;
    protected MedianFilter yawFilter;

    // Magic error scalar for radians to length
    protected double kP;

    protected int itemsAdded = 0;

    /**
     * Constructs a KugelMedianFilter
     * @param bucketSize odd value for the bucket of the median filter
     *                   If the value is odd it will be truncated
     */
    public KugelMedianFilter(int bucketSize) {
        head = 0;
        this.bucketSize = bucketSize;
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

        if (itemsAdded < bucketSize) {
            DriverStation.reportError("we did not read enough items", Constants.debugMode);
            return null;
        }

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

        // return the time and position of the best position estimate
        return new OdoPosition(
                new Pose2d(x[indexOfSmallestError], y[indexOfSmallestError], new Rotation2d(yaw[indexOfSmallestError])),
                time[indexOfSmallestError]);
    }

}
