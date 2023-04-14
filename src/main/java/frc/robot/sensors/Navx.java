package frc.robot.sensors;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Newman_Constants.Constants;

public class Navx {
    //Instance Handling
    private static Navx m_pInstance;

    private double angle = 0d;

    private static double zeroPitch = Constants.Balance.defaultPitchZero;

    public static Navx GetInstance() {
        if (m_pInstance == null) m_pInstance = new Navx();
        return m_pInstance;
    }

    //Create necessary objects
    private static AHRS m_navX;


    //Create and define all standard data types needed
    private static boolean m_bNavXPresent;

    private Navx() {
        try {
            //Connect to navX Gyro on MXP port.
            m_navX = new AHRS(SPI.Port.kMXP);
            m_bNavXPresent = true;
        } catch (Exception ex) {
            //If connection fails log the error and fall back to encoder based angles.
            String str_error = "Error instantiating navX from MXP: " + ex.getLocalizedMessage();
            DriverStation.reportError(str_error, true);
            m_bNavXPresent = false;
        }
    }

    public static void resetNavX(){
        m_navX.reset();
        m_navX.zeroYaw();
    }

    /**
     * Returns the current angle of the Navx. If the Navx is not present, will return -1.
     *
     * @return angle in degrees
     */
    public static double getAngle() {
        if (m_bNavXPresent) {
            return Math.IEEEremainder((m_navX.getAngle() + 360) * (Constants.kNavxReversed ? -1.0 : 1.0), 720); // converts navx heading angle infinite values to -360 to 360 even tho thats stupid
        }
        return -1;
    }

    /**
     * get the angle in radians
     * @return the angle as a rotation 2d (in radians)
     */
    public static Rotation2d getRotation() {
        return (m_bNavXPresent) ? m_navX.getRotation2d() : new Rotation2d(-1);
    }

    /**
     * Returns the current yaw of the Navx. If the Navx is not present, will return NaN.
     *
     * @return angle in degrees (-180, 180)
     */
    public static double getYaw() {
        if (m_bNavXPresent) {
            return m_navX.getYaw();
        }
        return Double.NaN;
    }
    
    /**
     * Returns the current pitch of the Navx. If the Navx is not present, will return NaN.
     *
     * @return angle in degrees (-180, 180)
     */
    public static double getPitch() {
        if (m_bNavXPresent) {
            return m_navX.getPitch();
        }
        return Double.NaN;
    }

    /**
     * Returns the rotation rate of the Navx along the Y axis (Pitch). If the Navx is not present, will return NaN.
     *
     * @return angle in degrees (-180, 180)
     */
    public static double getPitchVelocity() {
        if (m_bNavXPresent) {
            return m_navX.getRawGyroX();
        }
        return Double.NaN;
    }

    /**
     * Returns the current roll of the Navx. If the Navx is not present, will return NaN.
     *
     * @return angle in degrees (-180, 180)
     */
    public static double getRoll() {
        if (m_bNavXPresent) {
            return m_navX.getRoll();
        }
        return Double.NaN;
    }

    /**
     * Returns the rotation rate of the Navx along the Y axis (Roll). If the Navx is not present, will return NaN.
     *
     * @return angle in degrees (-180, 180)
     */
    public static double getRollVelocity() {
        if (m_bNavXPresent) {
            return m_navX.getRawGyroY();
        }
        return Double.NaN;
    }

    /**
     * Returns the current rate of change of the robots heading
     *
     * <p> getRate() returns the rate of change of the angle the robot is facing,
     * with a return of negative one if the gyro isn't present on the robot,
     * as calculating the rate of change of the angle using encoders is not currently being done.
     *
     * @return the rate of change of the heading of the robot in degrees per second.
     */
    public static double getRate() {
        if (m_bNavXPresent) return m_navX.getRate() * (Constants.kNavxReversed ? -1.0 : 1.0);
        return -1;
    }

    /**
     * Returns the current heading of the Navx. Range is wrapped onto 0 to 360
     * If the Navx is not present, will return -1.
     *
     * @return angle in degrees
     */
    public static double getHeading() {
        return Math.IEEEremainder(getAngle(), 360);
    }

    public static boolean getNavxPresent() {
        return m_bNavXPresent;
    }

    public static void setPitchZero(double val){
        zeroPitch = val;
    }

    public static double getZeroPitch(){
        return zeroPitch;
    }

    public static void outputToShuffleboard() {
        SmartDashboard.putNumber("NavX angle", getRotation().getDegrees());
        SmartDashboard.putNumber("NavX Yaw", getYaw());
        SmartDashboard.putNumber("NavX Pitch", getPitch());
        SmartDashboard.putNumber("NavX Roll", getRoll());
        
    }
}