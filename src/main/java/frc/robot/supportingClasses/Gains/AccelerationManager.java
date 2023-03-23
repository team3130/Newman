package frc.robot.supportingClasses.Gains;

/**
 * A class to manage calculating acceleration.
 * It can be used in two ways: with {@link #getAcceleration(double, double)} which updates the values
 *  and gets acceleration, or you can use:
 * {@link #update(double, double)} and then when you want to get the acceleration, use {@link #getAcceleration()}, this
 *  would frequently be used when you need to pass a lambda
 */
public class AccelerationManager {
    // last ticks velocity and time
    private double previousVelocity;
    private double previousTime;

    // only used if update is also used
    private double velocity;
    private double time;

    /**
     * Default constructor to make an acceleration manager object
     */
    public AccelerationManager() {
        previousTime = 0;
        previousVelocity = 0;
    }

    /**
     * Can not have update be called every tick
     * @param velocity the "current" velocity
     * @param time the current time
     * @return the "current" acceleration
     */
    public double getAcceleration(double velocity, double time) {
        final double toReturn = (velocity - previousVelocity) / (time - previousTime);
        previousVelocity = velocity;
        previousTime = time;
        return toReturn;
    }

    /**
     * Used with {@link #getAcceleration()} especially in cases where a lambda is needed.
     * MAKE SURE THIS IS CALLED BEFORE the other get acceleration.
     * @param velocity the "current" velocity
     * @param time the current time
     */
    public void update(double velocity, double time) {
        previousVelocity = this.velocity;
        previousTime = this.time;
        this.velocity = velocity;
        this.time = time;
    }

    /**
     * Requires update to be called every tick
     * @return the acceleration based off of {@link #update(double, double)} values
     */
    public double getAcceleration() {
        return (velocity - previousVelocity) / (time - previousTime);
    }
}
