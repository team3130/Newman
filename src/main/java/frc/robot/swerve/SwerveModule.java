package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Newman_Constants.Constants;

public class SwerveModule {
    private final WPI_TalonFX m_steerMotor;
    private final WPI_TalonFX m_driveMotor;

    private final CANCoder m_absoluteEncoder;

    private final PIDController turningPidController;
    private final double absoluteEncoderOffset;

    private final int side;

    public SwerveModule(int side) {
        m_steerMotor = new WPI_TalonFX(Constants.turningId[side]);
        m_driveMotor = new WPI_TalonFX(Constants.spinningId[side]);

        m_absoluteEncoder = new CANCoder(Constants.CANCoders[side]);

        turningPidController = new PIDController(Constants.SwerveKp, Constants.SwerveKi, Constants.SwerveKd);

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

        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        turningPidController.setTolerance(0.003);

        absoluteEncoderOffset = Constants.kCanCoderOffsets[side];

        this.side = side;

        resetEncoders();
    }

    public double getDrivePosition(){
        return m_driveMotor.getSelectedSensorPosition() * Constants.DriveTicksToMeters;
    }

    public double getTurningPosition(){
        return Constants.SteerTicksToRads * m_steerMotor.getSelectedSensorPosition(); //* Constants.SteerTicksToRads;
    }
    public double getDriveVelocity() {
        return m_driveMotor.getSelectedSensorVelocity() * Constants.DriveTicksToMetersPerSecond;
    }

    public double getTurningVelocity() {
        return m_steerMotor.getSelectedSensorVelocity() * Constants.SteerTicksToRadsPerSecond;
    }

    public double getAbsoluteEncoderRad() {
        return Math.toRadians(m_absoluteEncoder.getAbsolutePosition());
    }

    public double getAbsoluteEncoderDegrees() {
        return m_absoluteEncoder.getAbsolutePosition();
    }

    public void updatePValue(double p) {
        turningPidController.setP(p);
    }

    public void updateDValue(double d) {
        turningPidController.setD(d);
    }

    public void resetEncoders(){
        m_steerMotor.setSelectedSensorPosition((getAbsoluteEncoderRad() - absoluteEncoderOffset) / Constants.SteerTicksToRads);
        m_driveMotor.setSelectedSensorPosition(0);
    }

    public boolean wheelsZeroed() {
        Rotation2d pos = new Rotation2d(getTurningPosition());
        return (pos.getDegrees() > 355 || pos.getDegrees() < 5) && getTurningVelocity() < 0.05;
    }

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
    public void setDesiredState(SwerveModuleState state){
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // max turn is 90 degrees optimization
        state = SwerveModuleState.optimize(state, getState().angle);
       m_driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / Constants.kPhysicalMaxSpeedMetersPerSecond);

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
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningVelocity()));
    }

    /**
     * Whether the pid controller is at the setpoint
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
}
