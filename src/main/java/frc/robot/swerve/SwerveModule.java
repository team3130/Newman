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
import frc.robot.Constants;

public class SwerveModule {
    private final WPI_TalonFX m_steerMotor;
    private final WPI_TalonFX m_driveMotor;
    private final CANCoder m_absoluteEncoder;
    private final PIDController turningPidController;
    private final double absoluteEncoderOffset;

    private static ShuffleboardTab tab = Shuffleboard.getTab("Swerve Module");
    // private final GenericEntry nAbsEncoderReadingTicks;
    // private final GenericEntry nAbsEncoderReadingRads;
    // private final GenericEntry nPosToGetTo;
    // private final GenericEntry nRelEncoderReadingTicks;
    // private final GenericEntry nRelEncoderReadingRads;

    private final int side;

    public SwerveModule(int side) {
        m_steerMotor = new WPI_TalonFX(Constants.turningId[side]);
        m_driveMotor = new WPI_TalonFX(Constants.spinningId[side]);

        m_absoluteEncoder = new CANCoder(Constants.CANCoders[side]);

        // network stuffs
        // nAbsEncoderReadingTicks = tab.add("ticks abs encoder " + side, 0).getEntry();
        // nAbsEncoderReadingRads = tab.add("rads abs encoder " + side, 0).getEntry();
        // nPosToGetTo = tab.add("Target " + side, 0).getEntry();

        // nRelEncoderReadingTicks = tab.add("ticks rel encoder " + side, 0).getEntry();
        // nRelEncoderReadingRads = tab.add("rel encoder " + side, 0).getEntry();

        turningPidController = new PIDController(Constants.SwerveKp, Constants.SwerveKi, Constants.SwerveKd);

        m_steerMotor.configFactoryDefault();
        m_steerMotor.setNeutralMode(NeutralMode.Brake);
        m_steerMotor.configVoltageCompSaturation(Constants.kMaxSteerVoltage);
        m_steerMotor.enableVoltageCompensation(true);
        //m_steerMotor.configOpenloopRamp(Constants.openLoopRampRate);

        // m_steerMotor.setSensorPhase(true);
        m_steerMotor.setInverted(true);

        m_driveMotor.configFactoryDefault();
        m_driveMotor.setNeutralMode(NeutralMode.Brake);
        m_driveMotor.configVoltageCompSaturation(Constants.kMaxDriveVoltage);
        m_driveMotor.enableVoltageCompensation(true);
        //m_driveMotor.configOpenloopRamp(Constants.openLoopRampRate);
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

    public void outputToShuffleboard() {
        // nAbsEncoderReadingTicks.setDouble(getAbsolutEncoderTicks());
        // nAbsEncoderReadingRads.setDouble(getAbsoluteEncoderRad() - Constants.kCanCoderOffsets[side]);


        // nRelEncoderReadingTicks.setDouble(getTurningPosition());
        // nRelEncoderReadingRads.setDouble(getTurningPosition());
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

    public void stop(){
        m_steerMotor.set(ControlMode.PercentOutput, 0);
        m_driveMotor.set(ControlMode.PercentOutput, 0);
    }

    public void setDesiredState(SwerveModuleState state){
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        // nPosToGetTo.setDouble(turningPidController.calculate(Math.IEEEremainder(getTurningPosition(), Math.PI * 2), state.angle.getRadians()));
        // max turn is 90 degrees optimization
        state = SwerveModuleState.optimize(state, getState().angle);
        // nPosToGetTo.setDouble(state.angle.getRadians());
       m_driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / Constants.kPhysicalMaxSpeedMetersPerSecond);
//        m_driveMotor.set(0);

        m_steerMotor.set(turningPidController.calculate(Math.IEEEremainder(getTurningPosition(), Math.PI * 2), state.angle.getRadians()));
    }

    /**
     * Turns the motors to an angle
     * @param setpoint in radians
     */
    public void turnToAngle(double setpoint) {
        m_steerMotor.set(turningPidController.calculate(Math.IEEEremainder(getTurningPosition(), Math.PI * 2), setpoint));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningVelocity()));
    }

    public boolean PIDisDone() {
        return turningPidController.atSetpoint();
    }

}
