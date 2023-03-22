package frc.robot.supportingClasses.Gains;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;

import java.util.Arrays;

public class VelocityGainFilter {
    // the backbone data structure
    protected GainData[] data;

    // the size of the array
    protected final int bucketSize;

    // moving window variables
    protected int floor;
    protected int capacity;

    // whether we have gotten enough data to fill our array yet
    protected boolean hasFilled;

    /**
     * Makes a new circular array
     * @param bucketSize size of the bucket for the circular array
     */
    public VelocityGainFilter(int bucketSize) {
        data = new GainData[bucketSize];
        floor = 0;
        capacity = -1;
        this.bucketSize = bucketSize;
    }

    /**
     * @return an object that is the result of this instance
     */
    public StatsResult getResult() {
        /*GainData[] dataCopy = new GainData[bucketSize * 3];
        int shift = bucketSize;
        for (int i = floor; i < capacity; i++) {
            int index = (i - floor) + shift;
            if (dataCopy[index] == null) {
                dataCopy[index] = data[i % bucketSize];
            }
        }*/

        // get the median
        GainData[] dataCopy = Arrays.copyOf(data, data.length);
        Arrays.sort(dataCopy);
        final double median;
        if (dataCopy.length % 2 == 1) {
            median = dataCopy[dataCopy.length / 2].getVelocityGain();
        }
        else {
            median = (dataCopy[dataCopy.length / 2].getVelocityGain() + dataCopy[dataCopy.length / 2 + 1].getVelocityGain()) / 2;
        }

        // get the average
        double total = 0;
        for (GainData gainData : dataCopy) {
            total += gainData.getVelocityGain();
        }
        final double average = total / (double) dataCopy.length;

        // get the standard deviation
        double totalForStandardDeviation = 0;
        for (GainData gainData: dataCopy) {
            totalForStandardDeviation += Math.pow(gainData.getVelocityGain() - average, 2);
        }
        final double standardDeviation = totalForStandardDeviation / dataCopy.length;

        return new StatsResult(median, standardDeviation, average);
    }


    /**
     * Wrapper for {@link #add(GainData)}
     * @param velocity the velocity at the current instant
     * @param percentOutput the percent output at that instant
     * @param time the time at that instant
     */
    public void add(double velocity,double percentOutput, double time) {
        add(new GainData(velocity, percentOutput, time));
    }

    /**
     * the adder for our circular array
     * @param data new data to add to our data structure
     */
    public void add(GainData data) {
        this.data[++capacity % bucketSize] = data;
        if (hasFilled) {
            floor++;
        }
        else {
            if (floor + bucketSize == capacity - 1) {
                floor++;
                hasFilled = true;
            }
        }
    }

    /**
     * @return The current velocity
     */
    public double getCurrentVelocity() {
        return data[capacity % bucketSize].getVelocity();
    }


    /**
     * @return an array of each gain
     */
    public double[] getGains() {
        double[] gains = new double[bucketSize];
        for (int i = floor; i < capacity; i++) {
            gains[i - floor] = data[i % bucketSize].getVelocityGain();
        }
        return gains;
    }

    /**
     * get the accelerations
     * @return an array of the derivatives
     */
    public double[] accelerations() {
        double[] accelerations = new double[bucketSize - 1];
        for (int i = floor + 1; i < capacity; i++) {
            accelerations[i - (floor + 1)] = data[i % capacity].getAccelerationBetween(data[(i - 1) % capacity]);
        }
        return accelerations;
    }

    /**
     * You should wait to actually use the dervy derv methods until after we have filled the array
     * @return if you can use dervy derv methods
     */
    public boolean useAble() {
        return hasFilled;
    }

    /**
     * Has no check for out of bounds
     * @return the current acceleration
     */
    public double currentAcceleration() {
        return data[capacity % bucketSize].getAccelerationBetween(data[capacity % bucketSize]);
    }

    /**
     * @return whether the collection is empty
     */
    public boolean isEmpty() {
        return capacity < 0;
    }


    /**
     * A class to store results
     */
    protected class StatsResult implements Sendable {
        protected final double median;
        protected final double standardDeviation;
        protected final double average;

        /**
         * Creates a new stats result
         * @param median the median value
         * @param standardDeviation the standard deviation
         * @param average the average
         */
        public StatsResult(double median, double standardDeviation, double average) {
            this.median = median;
            this.standardDeviation = standardDeviation;
            this.average = average;

            SendableRegistry.addLW(this, "State result " + Timer.getFPGATimestamp());
        }

        /**
         * @return the median
         */
        public double getMedian() {
            return median;
        }

        /**
         * @return the standard deviation
         */
        public double getStandardDeviation() {
            return standardDeviation;
        }

        /**
         * @return the average
         */
        public double getAverage() {
            return average;
        }

        /**
         * @param builder sendable builder
         */
        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("Nerd Data");
            builder.addDoubleProperty("median", this::getMedian, null);
            builder.addDoubleProperty("mean", this::getAverage, null);
            builder.addDoubleProperty("standard deviation", this::getStandardDeviation, null);
        }
    }

}
