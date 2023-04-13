package frc.robot.supportingClasses.Gains;

import java.util.Objects;

public class GainData implements Cloneable, Comparable {
    protected final double velocity;
    protected final double percentOutput;
    protected final double time;

    /**
     * @param velocity      the velocity at the instant
     * @param percentOutput the percent output at that instant
     * @param time          the time at that instant
     */
    public GainData(double velocity, double percentOutput, double time) {
        this.velocity = velocity;
        this.percentOutput = percentOutput;
        this.time = time;
    }

    /**
     * In theory this times desired velocity should be the percent output we need to run at for that velocity
     * @return the velocity gain
     */
    public double getVelocityGain() {
        return percentOutput / velocity;
    }

    /**
     * @return The velocity from this data point
     */
    public double getVelocity() {
        return velocity;
    }

    /**
     * @return The time from this data point
     */
    public double getTime() {
        return time;
    }

    /**
     * @return the acceleration between two different GainData objects
     */
    public double getAccelerationBetween(GainData other) {
        return (this.velocity - other.velocity) / (this.time - other.time);
    }

    /**
     * @return a clone of the object
     */
    @Override
    public GainData clone() {
        return new GainData(velocity, percentOutput, time);
    }

    /**
     * @param o the object to be compared.
     * @return -1 for less than, 0 for same, and 1 for greater than
     */
    @Override
    public int compareTo(Object o) {
        if (o != null && getClass() == o.getClass()) {
            return Double.compare(this.getVelocityGain(), ((GainData) o).getVelocityGain());
        }
        throw new RuntimeException("The passed in object was either null or was not the same class as this.");
    }

    /**
     * Default equate that checks the fields
     * @param o the other object
     * @return whether they are the same
     */
    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        GainData gainData = (GainData) o;
        return Double.compare(gainData.velocity, velocity) == 0 && Double.compare(gainData.percentOutput, percentOutput) == 0 && Double.compare(gainData.time, time) == 0;
    }

    /**
     * @return an int of hashed fields: velocity, percentOutput, and time
     */
    @Override
    public int hashCode() {
        return Objects.hash(velocity, percentOutput, time);
    }
}
