// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;
import frc.robot.sensors.Navx;
import frc.robot.supportingClasses.OdoPosition;
import frc.robot.swerve.SwerveModule;

import java.util.Arrays;


public class Chassis extends SubsystemBase {
    /** The geometry of the swerve modules */
    private final SwerveDriveKinematics m_kinematics;
    /** The odometry object */
    private final SwerveDrivePoseEstimator m_odometry;

    /** A list of the swerve modules (should be four */
    private final SwerveModule[] modules;
    /** Makes sure that the Navx Gyro is initialized */
    private final Navx Gyro = Navx.GetInstance();

    /** Whether it is field relative or robot oriented drive */
    private boolean fieldRelative = true;

    private Field2d field2d;

    /**
     * Makes a chassis that starts at 0, 0, 0
     */
    public Chassis(){
      this (new Pose2d(), new Rotation2d());
    }

    /**
     * Makes a chassis with a starting position
     * @param startingPos the initial position to say that the robot is at
     * @param startingRotation the initial rotation of the bot
     */
    public Chassis(Pose2d startingPos, Rotation2d startingRotation) {
        m_kinematics = new SwerveDriveKinematics(Constants.moduleTranslations);
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(),
              new SwerveModulePosition(), new SwerveModulePosition()};

        modules = new SwerveModule[4];
        modules[Constants.Side.LEFT_FRONT] = new SwerveModule(Constants.Side.LEFT_FRONT);
        modules[Constants.Side.LEFT_BACK] = new SwerveModule(Constants.Side.LEFT_BACK);
        modules[Constants.Side.RIGHT_FRONT] = new SwerveModule(Constants.Side.RIGHT_FRONT);
        modules[Constants.Side.RIGHT_BACK] = new SwerveModule(Constants.Side.RIGHT_BACK);

        zeroHeading();

        // odometry wrapper class that has functionality for cameras that report position with latency
        m_odometry = new SwerveDrivePoseEstimator(m_kinematics, startingRotation, generatePoses(), startingPos);

        field2d = new Field2d();
        Shuffleboard.getTab("Test").add(field2d);
    }

    /**
     * If the PID controllers of the {@link SwerveModule}'s are all done
     * @return whether the wheels are zereod/PID controllers are done
     */
    public boolean turnToAnglePIDIsDone() {
        return modules[Constants.Side.LEFT_FRONT].PIDisDone() &&
        modules[Constants.Side.LEFT_BACK].PIDisDone() &&
        modules[Constants.Side.RIGHT_FRONT].PIDisDone() &&
        modules[Constants.Side.RIGHT_BACK].PIDisDone();
    }

    /**
    * Resets odometry
    * <p>Resets navx</p>
    * <p>Resets relative encoders to be what the absolute encoders are</p>
    * <p>Hard reset of the odometry object</p>
    */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        Navx.resetNavX();
        m_odometry.resetPosition(getRotation2d(), generatePoses(), pose);
    }

    /**
     * Flip-flops between field relative and bot relative swerve drive
     */
    public void flipFieldRelative() {
      fieldRelative = !fieldRelative;
    }

    /**
     * Getter for if swerve drive is field relative or not
     * @return bool if field relative
     */
    public boolean getFieldRelative() {
      return fieldRelative;
    }

    /**
     * Zereos the Navx's heading
     */
    public void zeroHeading(){
      Navx.resetNavX();
    }

    /**
     * Returns the heading that navx reads
     * @return the rotation of the bot in degrees
     */
    public double getHeading() {
      return Math.IEEEremainder(Navx.getAngle(), 360);
    }

    /**
     * Returns the bots rotation according to Navx as a {@link Rotation2d}
     * @return the bot rotation
     */
    public Rotation2d getRotation2d(){
      return Rotation2d.fromDegrees(getHeading());
    }

    /**
     * Generates the positions of the swerve modules
     * @return the poses of each module
     */
    public SwerveModulePosition[] generatePoses() {
           return new SwerveModulePosition[]{
              modules[Constants.Side.LEFT_FRONT].getPosition(),
              modules[Constants.Side.LEFT_BACK].getPosition(),
              modules[Constants.Side.RIGHT_FRONT].getPosition(),
              modules[Constants.Side.RIGHT_BACK].getPosition()
      };
    }

    /**
     * periodic call to update odometry from encoders
     * Also provides a timestamp that the update occurred
     */
    public void updateOdometryFromSwerve() {
      m_odometry.updateWithTime(Timer.getFPGATimestamp(), Navx.getRotation(), generatePoses());
    }


    /**
     * subsystem looped call made by the scheduler.
     * Updates the odometry from swerve
     */
    @Override
    public void periodic() {
        updateOdometryFromSwerve();
        field2d.setRobotPose(m_odometry.getEstimatedPosition());
    }

  /**
   * Stops the devices connected to this subsystem
   */
  public void stopModules(){
      modules[Constants.Side.LEFT_FRONT].stop();
      modules[Constants.Side.LEFT_BACK].stop();
      modules[Constants.Side.RIGHT_FRONT].stop();
      modules[Constants.Side.RIGHT_BACK].stop();
  }

    /**
     * Getter for geometry
     * @return the geometry of the swerve modules
     */
    public SwerveDriveKinematics getKinematics() {
      return m_kinematics;
    }

    /**
     * Sets the module states to desired states
     * @param desiredStates the states to set the modules to
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kPhysicalMaxSpeedMetersPerSecond);

      modules[Constants.Side.LEFT_FRONT].setDesiredState(desiredStates[Constants.Side.LEFT_FRONT]);
      modules[Constants.Side.LEFT_BACK].setDesiredState(desiredStates[Constants.Side.LEFT_BACK]);
      modules[Constants.Side.RIGHT_FRONT].setDesiredState(desiredStates[Constants.Side.RIGHT_FRONT]);
      modules[Constants.Side.RIGHT_BACK].setDesiredState(desiredStates[Constants.Side.RIGHT_BACK]);
    }

    /**
     * Spins the wheels to an angle
     * @param setpoint angle to spin the motors to
     */
    public void turnToAngle(double setpoint) {
      for (SwerveModule module : modules) {
          module.turnToAngle(setpoint);
      }
    }

    /**
     * The simulation periodic call
     */
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    /**
     * Returns the current position of the bot as a Pose2d
     * @return position according to odometry
     */
    public Pose2d getPose2d() {
        return m_odometry.getEstimatedPosition();
    }

    /**
     * Command to reset the encoders
     */
    public void resetEncoders() {
        for (SwerveModule module : modules) {
            module.resetEncoders();
        }
    }

    /**
     * Sets the bot to be field relative
     */
    public void setFieldRelative() {
        fieldRelative = true;
    }

    /**
     * Sets the bot to robot oriented
     */
    public void setRobotOriented() {
        fieldRelative = false;
    }

    /**
     * Sets field oriented to the provided boolean
     */
    public void setWhetherFieldOriented(boolean fieldOriented) {
        fieldRelative = fieldOriented;
    }

    /**
     * update the P values for the swerve module
     */
    public void updatePValuesFromSwerveModule(double pValue) {
        Arrays.stream(modules).forEach((SwerveModule modules) -> modules.updatePValue(pValue));
    }

    /**
     * update the D values for the swerve module
     */
    public void updateDValuesFromSwerveModule(double dValue) {
        Arrays.stream(modules).forEach((SwerveModule modules) -> modules.updateDValue(dValue));
    }

    /**
     * gets the kP values for each module
     * @return gets the kP value from the modules
     */
    public double getPValuesForSwerveModules() {
        return modules[0].getPValue();
    }

    /**
     * gets the kD values for each module
     * @return gets the kD value from the modules
     */
    public double getDValuesForSwerveModules() {
        return modules[0].getDValue();
    }

    /**
     * @return the x position from odometry
     */
    private double getX() {
        return m_odometry.getEstimatedPosition().getX();
    }

    /**
     * @return the y position from odometry
     */
    private double getY() {
        return m_odometry.getEstimatedPosition().getY();
    }

    /**
     * @return the yaw from odometry
     */
    private double getYaw() {
        return m_odometry.getEstimatedPosition().getRotation().getDegrees();
    }

    /**
     * Initializes the data we send on shuffleboard
     * Calls the default init sendable for Subsystem Bases
     * @param builder sendable builder
     */
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Chassis");

        // add each submodule as a child
        Arrays.stream(modules).forEach((SwerveModule module) -> this.addChild(module.toString(), module));

        // add field relative
        builder.addBooleanProperty("fieldRelative", this::getFieldRelative, this::setWhetherFieldOriented);
        builder.addDoubleProperty("pSwerveModule", this::getPValuesForSwerveModules, this::updatePValuesFromSwerveModule);
        builder.addDoubleProperty("dSwerveModule", this::getDValuesForSwerveModules, this::updateDValuesFromSwerveModule);
        builder.addDoubleProperty("Navx", this::getHeading, null);
        builder.addDoubleProperty("X position", this::getX, null);
        builder.addDoubleProperty("Y position", this::getY, null);
        builder.addDoubleProperty("rotation", this::getYaw, null);
    }

    public void updateOdometryFromAprilTags(OdoPosition refreshPosition) {
        m_odometry.addVisionMeasurement(refreshPosition.getPosition(), refreshPosition.getTime());
    }
}