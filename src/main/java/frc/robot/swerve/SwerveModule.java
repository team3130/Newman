package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import frc.robot.Newman_Constants.Constants;


/**
 * Swerve module that reflects the actual swerve modules
 */
public class SwerveModule implements Sendable {
    private final WPI_TalonFX m_steerMotor; // the steering motor
    private final WPI_TalonFX m_driveMotor; // the driving motor

    private final CANCoder m_absoluteEncoder; // the can encoder attached to the shaft

    private final PIDController turningPidController; // PID controller for steering

    private final double absoluteEncoderOffset; // the absolute encoder offset from where 0 is to where it thinks it is

    private final int side; // the side that the bot is on

    /**
     * Initializes a swerve module and its motors.
     * Initializes the steering PID controller.
     * @param side is reflective in {@link Constants}
     */
    public SwerveModule(int side) {
        m_steerMotor = new WPI_TalonFX(Constants.turningId[side]);
        m_driveMotor = new WPI_TalonFX(Constants.spinningId[side]);

        m_absoluteEncoder = new CANCoder(Constants.CANCoders[side]);

        turningPidController = new PIDController(Constants.SwerveKp[side], Constants.SwerveKi[side], Constants.SwerveKd[side]);

        m_steerMotor.configFactoryDefault();
        m_steerMotor.setNeutralMode(NeutralMode.Brake);
        m_steerMotor.configVoltageCompSaturation(Constants.kMaxSteerVoltage);
        m_steerMotor.enableVoltageCompensation(true);
        m_steerMotor.setInverted(true);

        m_driveMotor.configFactoryDefault();
        m_driveMotor.setNeutralMode(NeutralMode.Brake);
        m_driveMotor.configVoltageCompSaturation(Constants.kMaxDriveVoltage);
        m_driveMotor.enableVoltageCompensation(true);
        m_driveMotor.setInverted(false);

        turningPidController.enableContinuousInput(-Math.PI, Math.PI); // wrap for circles
        turningPidController.setTolerance(0.0025, 0.05); // at position tolerance

        absoluteEncoderOffset = Constants.kCanCoderOffsets[side];

        this.side = side;

        resetEncoders();

        String name = this.getClass().getSimpleName();
        name = name.substring(name.lastIndexOf('.') + 1);
        name += " " + side;
        SendableRegistry.addLW(this, name, name);
    }

    /**
     * @return the amount of distance that the drive motor has traveled in meters
     */
    public double getDrivePosition(){
        return m_driveMotor.getSelectedSensorPosition() * Constants.DriveTicksToMeters;
    }

    /**
     * @return The position of the steering motor radians
     */
    public double getTurningPosition(){
        return Constants.SteerTicksToRads * m_steerMotor.getSelectedSensorPosition(); //* Constants.SteerTicksToRads;
    }

    /**
     * @return gets the velocity of the drive motor in m/s
     */
    public double getDriveVelocity() {
        return m_driveMotor.getSelectedSensorVelocity() * Constants.DriveTicksToMetersPerSecond;
    }

    /**
     * @return gets the speed at which the steering motor turns in radians per second
     */
    public double getTurningVelocity() {
        return m_steerMotor.getSelectedSensorVelocity() * Constants.SteerTicksToRadsPerSecond;
    }

    /**
     * @return gets the position of the steering wheel according to the absolute encoders
     */
    public double getAbsoluteEncoderRad() {
        return Math.toRadians(m_absoluteEncoder.getAbsolutePosition());
    }

    /**
     * @return the position of the steering wheel in degrees
     */
    public double getAbsoluteEncoderDegrees() {
        return m_absoluteEncoder.getAbsolutePosition();
    }

    /**
     * updates the steering PID controller
     * @param p the new kP value
     */
    public void updatePValue(double p) {
        turningPidController.setP(p);
    }

    /**
     * updates the steering PID controller
     * @param d the new kD value
     */
    public void updateDValue(double d) {
        turningPidController.setD(d);
    }

    /**
     * Resets the relative encoders according the absolute encoder involving the offset
     */
    public void resetEncoders() {
        m_steerMotor.setSelectedSensorPosition((getAbsoluteEncoderRad() - absoluteEncoderOffset) / Constants.SteerTicksToRads);
        // m_driveMotor.setSelectedSensorPosition(0);
    }

    /**
     * Whether the wheels are zeroed or not
     * @return custom at set-point logic for the PID controller
     */
    public boolean wheelsZeroed() {
        Rotation2d pos = new Rotation2d(getTurningPosition());
        return (pos.getDegrees() > 355 || pos.getDegrees() < 5) && getTurningVelocity() < 0.05;
    }

    /**
     * @return the current swerve module state
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    /**
     * Default stop method to stop the motors
     */
    public void stop(){
        m_steerMotor.set(ControlMode.PercentOutput, 0);
        m_driveMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Set the desired swerve module state
     * @param state the state to set the swerve modules to
     */
    public void setDesiredState(SwerveModuleState state) {
        // dead-band
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // max turn is 90 degrees optimization
        state = SwerveModuleState.optimize(state, getState().angle);
        // percent output of the drive motor that the swerve controller wants you to go to by the physical max speed the bot can travel
        m_driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / Constants.kPhysicalMaxSpeedMetersPerSecond);
        // set the steering motor based off the output of the PID controller
        m_steerMotor.set(turningPidController.calculate(Math.IEEEremainder(getTurningPosition(), Math.PI * 2), state.angle.getRadians()));
    }

    /**
     * Turns the motors to an angle
     * @param setpoint in radians
     */
    public void turnToAngle(double setpoint) {
        m_steerMotor.set(turningPidController.calculate(Math.IEEEremainder(getTurningPosition(), Math.PI * 2), setpoint));
    }

    /**
     * Get the position of the swerve module
     * @return gets with turning position and velocity
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    /**
     * Whether the pid controller is at the set point.
     * @return whether pid is done
     */
    public boolean PIDisDone() {
        return turningPidController.atSetpoint();
    }

    /**
     * Gets the P value for steering motors
     * @return the P value
     */
    public double getPValue() {
        return turningPidController.getP();
    }

    /**
     * Gets the P value for steering motors
     * @return the P value
     */
    public double getDValue() {
        return turningPidController.getD();
    }

    /**
     * Setter for the P value
     * @param newP the new p value
     */
    public void setPValue(double newP) {
        turningPidController.setP(newP);
    }

    /**
     * Setter for dervy derv
     * @param newD the new D value
     */
    public void setDValue(double newD) {
        turningPidController.setD(newD);
    }

    public void setIValue(double newI) {
        turningPidController.setI(newI);
    }

    public double getIValue() {
        return turningPidController.getI();
    }

    /**
     * The string representation of the swerve module
     * @return "Swerve module side: " + sideNumber: int
     */
    public String toString() {
        return "Swerve module side: " + side;
    }

    /**
     * Builds the sendable for shuffleboard
     * @param builder sendable builder
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Swerve Module " + side);
/*        builder.addDoubleProperty("Drive position", this::getDrivePosition, null);
        builder.addDoubleProperty("Drive velocity", this::getDriveVelocity, null);*/
        builder.addDoubleProperty("Steer position", this::getSteerPositionWrapped, null);
/*        builder.addDoubleProperty("Steer velocity", this::getTurningVelocity, null);
        builder.addDoubleProperty("Steer relative", this::getRelativePositionDegrees, null);
        builder.addDoubleProperty("Absolute encoder position", this::getAbsoluteEncoderDegrees, null);*/
        builder.addDoubleProperty("Swerve P " + side, this::getPValue, this::setPValue);
        builder.addDoubleProperty("Swerve I " + side, this::getIValue, this::setIValue);
        builder.addDoubleProperty("Swerve D " + side, this::getDValue, this::setDValue);
    }

    public double getSteerPositionWrapped() {
        return Math.IEEEremainder(getRelDegrees(), 360);
    }

    public double getRelativePositionDegrees() {
        return Math.toDegrees(getTurningPosition());
    }

    public double getRelDegrees() {
        return Math.toDegrees(getTurningPosition());
    }
}
