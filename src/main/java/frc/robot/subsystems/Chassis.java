// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Newman_Constants.Constants;
import frc.robot.sensors.Limelight;
import frc.robot.sensors.Navx;
import frc.robot.supportingClasses.Auton.HolonomicControllerCommand;
import frc.robot.supportingClasses.Vision.OdoPosition;
import frc.robot.swerve.SwerveModule;

import java.util.Arrays;

/**
 * Chassis is the drivetrain subsystem of our bot. Our physical chassis is a swerve drive, 
 * so we use wpilib SwerveDriveKinematics and SwerveDrivePoseEstimator as opposed to Differential Drive objects
 */
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

    /** limelight object */
    private final Limelight m_limelight;

    /**
     * Updated periodically with the maximum speed that has been read on any of the swerve modules
     */
    private final ProfiledPIDController HolonomicPIDController;
    private final PIDController HoldController;
    private Boolean useHold;
    private double currentGoal = 0;
    private double maxSpeedRead = 0;

    /**
     * A sendable that gets put on shuffleboard with the auton trajectory and the robots current position
     */
    private final Field2d field;

    /**
     * A comp network table entry for whether drivetrain is in field oriented or not
     */
    private final GenericEntry n_fieldOrriented;

    /**
     * Whether to update the odometry with the april tag or not.
     * Usuallt used as a toggleable in auton commands.
     * Default value is {@link Constants#useAprilTags} however is mutable.
     */
    protected boolean useAprilTags = Constants.useAprilTags;

    /**
     * Makes a chassis that starts at 0, 0, 0
     * @param limelight the limelight object that we can use for updating odometry
     */
    public Chassis(Limelight limelight){
      this (new Pose2d(), new Rotation2d(), limelight);
    }

    /**
     * Makes a chassis with a starting position
     * @param startingPos the initial position to say that the robot is at
     * @param startingRotation the initial rotation of the bot
     * @param limelight the limelight object which is used for updating odometry
     */
    public Chassis(Pose2d startingPos, Rotation2d startingRotation, Limelight limelight) {
        m_kinematics = new SwerveDriveKinematics(Constants.moduleTranslations);

        modules = new SwerveModule[4];
        modules[Constants.Side.LEFT_FRONT] = new SwerveModule(Constants.Side.LEFT_FRONT);
        modules[Constants.Side.LEFT_BACK] = new SwerveModule(Constants.Side.LEFT_BACK);
        modules[Constants.Side.RIGHT_FRONT] = new SwerveModule(Constants.Side.RIGHT_FRONT);
        modules[Constants.Side.RIGHT_BACK] = new SwerveModule(Constants.Side.RIGHT_BACK);

        // odometry wrapper class that has functionality for cameras that report position with latency
        m_odometry = new SwerveDrivePoseEstimator(m_kinematics, startingRotation, generatePoses(), startingPos);

        m_limelight = limelight;

        field = new Field2d();
        Shuffleboard.getTab("Comp").add("field", field);
        n_fieldOrriented = Shuffleboard.getTab("Comp").add("field orriented", false).getEntry();

        HolonomicPIDController = new ProfiledPIDController(0.5, Constants.kIThetaController, Constants.kDThetaController, Constants.kThetaControllerConstraints);
        HoldController = new PIDController(0.25, 0, 0.0025);

        HolonomicPIDController.setTolerance(Math.toRadians(2.5));
        HoldController.setTolerance(0.025);
        useHold = false;
  }

    public void SetHoloGoal(double angle) {
        currentGoal = angle;
        HolonomicPIDController.reset(getHeading());
        HolonomicPIDController.setGoal(angle);
        useHold = false;
    }

    public void holdHoloAt(double angle) {
        currentGoal = angle;
        HoldController.setSetpoint(angle);
        useHold = true;
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
     * @param pose the position to reset odometry to
    */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        Navx.resetNavX();
        m_odometry.resetPosition(Navx.getRotation(), generatePoses(), pose);
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
     * Zeros the Navx's heading
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
      return m_odometry.getEstimatedPosition().getRotation();
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
     * Updates the odometry from vision if there is a new value to update position with
     */
    public void updateOdometryFromVision() {
        OdoPosition position = refreshPosition();
        if (position != null) {
            updateOdometryFromVision(position);
        }
    }

    /**
     * Update odometry with swerve drive. Also updates odometry with vision if the {@link Constants#useAprilTags}'s flag true
     */
    public void updateOdometery() {
        updateOdometryFromSwerve();
        if (Constants.useAprilTags && useAprilTags) {
            updateOdometryFromVision();
        }
    }


    /**
     * subsystem looped call made by the scheduler.
     * Updates the odometry from swerve and April Tags.
     * Also updates and sendables we use during comp
     */
    @Override
    public void periodic() {
        n_fieldOrriented.setBoolean(fieldRelative);
        field.setRobotPose(m_odometry.getEstimatedPosition());
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
     * Returns the current position of the bot as a {@link Pose2d}
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
     * @param fieldOriented to drive in field or robot orriented
     */
    public void setWhetherFieldOriented(boolean fieldOriented) {
        fieldRelative = fieldOriented;
    }

    /**
     * update the P values for the swerve module
     * @param pValue the new P value
     */
    public void updatePValuesFromSwerveModule(double pValue) {
        Arrays.stream(modules).forEach((SwerveModule modules) -> modules.updatePValue(pValue));
    }

    /**
     * update the D values for the swerve module
     * @param dValue the new D value
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
     * A vomit onto shuffleboard of the {@link SwerveModule} objects in Chassis
     * @param tab the tab to add the {@link SwerveModule} objects
     */
    public void shuffleboardVom(ShuffleboardTab tab) {
        tab.add(modules[0]);
        tab.add(modules[1]);
        tab.add(modules[2]);
        tab.add(modules[3]);
    }

    /**
     * Initializes the data we send on shuffleboard
     * Calls the default init sendable for Subsystem Bases
     * @param builder sendable builder
     */
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Chassis");

        // add field relative
        builder.addBooleanProperty("fieldRelative", this::getFieldRelative, this::setWhetherFieldOriented);
        builder.addDoubleProperty("Navx", this::getHeading, null);
        builder.addDoubleProperty("X position", this::getX, null);
        builder.addDoubleProperty("Y position", this::getY, null);
        builder.addDoubleProperty("rotation", this::getYaw, null);
        builder.addDoubleProperty("max speed read", this::getMaxSpeedRead, null);
    }

    /**
     * A listener to calculate what the max speed we read was
     */
    public void listener() {
        for (SwerveModule module : modules) {
            if (maxSpeedRead < module.getDriveVelocity()) {
                maxSpeedRead = module.getDriveVelocity();
            }
        }
    }

    /**
     * Gets the max speed field
     * @return the max speed that we read thus far on this vm instance of rio
     */
    public double getMaxSpeedRead() {
        return maxSpeedRead;
    }

    /**
     * update odometry from april tags
     * @param refreshPosition time and position to set to
     */
    public void updateOdometryFromVision(OdoPosition refreshPosition) {
        m_odometry.addVisionMeasurement(
            new Pose2d(
                refreshPosition.getPosition().getTranslation(), 
                refreshPosition.getPosition().getRotation()), 
            refreshPosition.getTime()
        );
    }

    /**
     * Refreshes the position from limelight and it's median filter
     * @return the odoPosition from limelight
     */
    public OdoPosition refreshPosition() {
        return m_limelight.calculate();
  }

  /**
   * updates the field object with a trajectory
   * @param trajectory the trajectory to set the field object wtih
   */
    public void updateField2DFromTrajectory(PathPlannerTrajectory trajectory) {
        field.getObject("traj").setTrajectory(trajectory);
    }

    /**
     * The same as {@link #drive(double, double, double)} except you pass in if you are field relative or not.
     * This method will drive the swerve modules based to x, y and theta vectors.
     * @param x velocity in the x dimension m/s
     * @param y velocity in the y dimension m/s
     * @param theta the angular (holonomic) speed to drive the swerve modules at
     * @param fieldRelative whether to use
     */
    public void drive(double x, double y, double theta, boolean fieldRelative) {
        if (fieldRelative) {
            setModuleStates(m_kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, theta, getRotation2d())));
        }
        else {
            setModuleStates(m_kinematics.toSwerveModuleStates(new ChassisSpeeds(x, y, theta)));
        }
    }

    public void drive(double x, double y) {
        double theta;
        if (useHold) {
            theta = HoldController.calculate(getHeading());
        }
        else {
            theta = HolonomicPIDController.calculate(getHeading());
        }
        drive(x, y, theta);
    }

    /**
     * Our main method to drive using three variables. Locked to field relative or robot oriented based off of {@link #fieldRelative}.
     * @param x the velocity in the x dimension m/s
     * @param y the velocity in the y dimension m/s
     * @param theta the angular (holonomic) speed of the bot
     */
    public void drive(double x, double y, double theta) {
        drive(x, y, theta, getFieldRelative());
    }

    /**
     * setter for april tags
     * @param useAprilTags whether to use april tags or not
     */
    public void setAprilTagUsage(boolean useAprilTags) {
        this.useAprilTags = useAprilTags;
    }
}