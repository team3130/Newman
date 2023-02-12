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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.Navx;
import frc.robot.swerve.SwerveModule;
import frc.robot.Newman_Constants.Constants;


public class Chassis extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final SwerveDriveKinematics m_kinematics;
  private final SwerveDrivePoseEstimator m_odometry;

  private SwerveModulePosition[] modulePositions;
  private SwerveModule[] modules;
  private final Navx Gyro = Navx.GetInstance();

  private static ShuffleboardTab tab = Shuffleboard.getTab("Chassis");

  private final GenericEntry Kp = tab.add("p", Constants.SwerveKp).getEntry();
  private final GenericEntry Kd = tab.add("d", Constants.SwerveKd).getEntry();
    private double lastKpRead = Constants.SwerveKp;
  private double lastKdRead = Constants.SwerveKd;

  private final GenericEntry n_FieldRelative = tab.add("field relative", true).getEntry();
  private final GenericEntry n_Navx;
  private final Field2d nField;

  private boolean fieldRelative = true;

  public Chassis(){
      this (new Pose2d(), new Rotation2d());
  }

  public Chassis(Pose2d startingPos, Rotation2d startingRotation) {
      m_kinematics = new SwerveDriveKinematics(Constants.moduleTranslations);
      modulePositions = new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(),
              new SwerveModulePosition(), new SwerveModulePosition()};

      // odometry wrapper class that has functionality for cameras that report position with latency
      m_odometry = new SwerveDrivePoseEstimator(m_kinematics, startingRotation, modulePositions, startingPos);

      modules = new SwerveModule[4];
      modules[Constants.Side.LEFT_FRONT] = new SwerveModule(Constants.Side.LEFT_FRONT);
      modules[Constants.Side.LEFT_BACK] = new SwerveModule(Constants.Side.LEFT_BACK);
      modules[Constants.Side.RIGHT_FRONT] = new SwerveModule(Constants.Side.RIGHT_FRONT);
      modules[Constants.Side.RIGHT_BACK] = new SwerveModule(Constants.Side.RIGHT_BACK);

      zeroHeading();

      n_Navx = tab.add("Navx angle", Navx.getRotation().getRadians()).getEntry();

      nField = new Field2d();
      
      SmartDashboard.putData(nField);
  }

    public boolean wheelsAreZeroed() {
        return modules[Constants.Side.LEFT_FRONT].PIDisDone() &&
        modules[Constants.Side.LEFT_BACK].PIDisDone() &&
        modules[Constants.Side.RIGHT_FRONT].PIDisDone() &&
        modules[Constants.Side.RIGHT_BACK].PIDisDone();
    }

  public void flipBool() {
      fieldRelative = !fieldRelative;
  }

  public boolean getFieldRelative() {
      return fieldRelative;
  }
  
  public void zeroHeading(){
      Navx.resetNavX();
  }

  public double getHeading() {
      return Math.IEEEremainder(Navx.getAngle(), 360);
  }

  public Rotation2d getRotation2d(){
      return Rotation2d.fromDegrees(getHeading());
  }

  public void outputToShuffleboard() {
      if (lastKpRead != Kp.getDouble(lastKpRead) ){
          lastKpRead = Kp.getDouble(lastKpRead);
          modules[Constants.Side.LEFT_FRONT].updatePValue(lastKpRead);
          modules[Constants.Side.LEFT_BACK].updatePValue(lastKpRead);
          modules[Constants.Side.RIGHT_FRONT].updatePValue(lastKpRead);
          modules[Constants.Side.RIGHT_BACK].updatePValue(lastKpRead);
      }
      if (lastKdRead != Kd.getDouble(lastKdRead) ){
          lastKdRead = Kd.getDouble(lastKdRead);
          modules[Constants.Side.LEFT_FRONT].updateDValue(lastKdRead);
          modules[Constants.Side.LEFT_BACK].updateDValue(lastKdRead);
          modules[Constants.Side.RIGHT_FRONT].updateDValue(lastKdRead);
          modules[Constants.Side.RIGHT_BACK].updateDValue(lastKdRead);
      }

      n_FieldRelative.setBoolean(fieldRelative);

      n_Navx.setDouble(Navx.getAngle());

      nField.setRobotPose(m_odometry.getEstimatedPosition());
  }

  public SwerveModulePosition[] generatePoses() {
           return new SwerveModulePosition[]{
              modules[Constants.Side.LEFT_FRONT].getPosition(),
              modules[Constants.Side.LEFT_BACK].getPosition(),
              modules[Constants.Side.RIGHT_FRONT].getPosition(),
              modules[Constants.Side.RIGHT_BACK].getPosition()
      };
  }

  public void updateOdometryFromSwerve() {
      m_odometry.updateWithTime(Timer.getFPGATimestamp(), Navx.getRotation(), generatePoses());
  }

  @Override
  public void periodic() {
    updateOdometryFromSwerve();

      outputToShuffleboard();

      modules[Constants.Side.LEFT_FRONT].outputToShuffleboard();
      modules[Constants.Side.LEFT_BACK].outputToShuffleboard();
      modules[Constants.Side.RIGHT_FRONT].outputToShuffleboard();
      modules[Constants.Side.RIGHT_BACK].outputToShuffleboard();
  }

  public void stopModules(){
      modules[Constants.Side.LEFT_FRONT].stop();
      modules[Constants.Side.LEFT_BACK].stop();
      modules[Constants.Side.RIGHT_FRONT].stop();
      modules[Constants.Side.RIGHT_BACK].stop();
  }

  public SwerveDriveKinematics getKinematics() {
      return m_kinematics;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kPhysicalMaxSpeedMetersPerSecond);

      modules[Constants.Side.LEFT_FRONT].setDesiredState(desiredStates[Constants.Side.LEFT_FRONT]);
      modules[Constants.Side.LEFT_BACK].setDesiredState(desiredStates[Constants.Side.LEFT_BACK]);
      modules[Constants.Side.RIGHT_FRONT].setDesiredState(desiredStates[Constants.Side.RIGHT_FRONT]);
      modules[Constants.Side.RIGHT_BACK].setDesiredState(desiredStates[Constants.Side.RIGHT_BACK]);
  }

  public void turnToAngle(double setpoint) {
      for (SwerveModule module : modules) {
          module.turnToAngle(setpoint);
      }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

    public void resetEncoders() {
      for (int i = 0; i < modules.length; i++) {
          modules[i].resetEncoders();
      }
    }
}