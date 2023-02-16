// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.FlipFieldOrriented;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.ZeroEverything;
import frc.robot.commands.ZeroWheels;
import frc.robot.subsystems.Chassis;
import frc.robot.Newman_Constants.Constants;
import frc.robot.supporting_classes.Chooser;
import frc.robot.supporting_classes.KugelControllerCommand;

import java.io.IOException;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  private static Joystick m_driverGamepad;
  private final Chassis m_chassis = new Chassis();
  private Command auton_command;


  public Chassis getChassis() {
    return m_chassis;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    m_driverGamepad = new Joystick(0);
    configureButtonBindings();

     m_chassis.setDefaultCommand(new TeleopDrive(m_chassis));
  }

  public static Joystick getDriverGamepad() {
    return m_driverGamepad;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverGamepad, Constants.Buttons.LST_BTN_A).whileTrue(new ZeroWheels(m_chassis));
    new JoystickButton(m_driverGamepad, Constants.Buttons.LST_BTN_B).whileTrue(new ZeroEverything(m_chassis));
    SmartDashboard.putData(new FlipFieldOrriented(m_chassis));
  }
  public Command generateAutonCommand() {
    Trajectory trajectory3;
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.kPhysicalMaxSpeedMetersPerSecond / 4, Constants.kMaxAccelerationDrive / 8)
            .setKinematics(m_chassis.getKinematics());
/*   Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0,0, new Rotation2d(0)), List.of(new Translation2d(0.5, 0)),
            new Pose2d(1,0, new Rotation2d(Math.toRadians(45))), trajectoryConfig);*/
   /* Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(List.of(
            new Pose2d(0, 0, new Rotation2d(0)),
            new Pose2d(2, 0, new Rotation2d(Math.toRadians(0)))
    ), trajectoryConfig);
    Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(3, 0, new Rotation2d(Math.toRadians(90)))), trajectoryConfig);*/
    /*    Trajectory trajectory3 = new Trajectory();*/

/*
    PathPlannerTrajectory trajectoryPlanned = PathPlanner.loadPath("3meter", new PathConstraints(4, 3));
*/
    PathPlannerTrajectory trajectoryPlanned = PathPlanner.generatePath(new PathConstraints(2, 2), List.of(
            new PathPoint(new Translation2d(0, 0), new Rotation2d(0), new Rotation2d(0)),
            new PathPoint(new Translation2d(3, 0), new Rotation2d(Math.toRadians(0)), new Rotation2d(Math.toRadians(90)))));
   /* PathPlannerTrajectory trajectoryPlanned = PathPlanner.generatePath(new PathConstraints(2,2), List.of(
            new PathPoint(new Translation2d(0,0), new Rotation2d(0), new Rotation2d(0) ),
            new PathPoint(new Translation2d(1.5, 0.1), new Rotation2d(Math.toRadians(30)), new Rotation2d(0)), new PathPoint(new Translation2d(3,1),
                    new Rotation2d(Math.toRadians(90)), new Rotation2d(0)), new PathPoint(new Translation2d(2.3,1.5),  new Rotation2d(Math.toRadians(120)), new Rotation2d(0)),
                    new PathPoint( new Translation2d(1,2),  new Rotation2d(Math.toRadians(180)), new Rotation2d(0)),
                    new PathPoint(new Translation2d(1.5, 2.2), new Rotation2d(0), new Rotation2d(0)),
                    new PathPoint(new Translation2d(2.25, 2.5), new Rotation2d(Math.toRadians(30)), new Rotation2d(0)),
                    new PathPoint(new Translation2d(3,3.25),  new Rotation2d(Math.toRadians(90)), new Rotation2d(0)),
                    new PathPoint(new Translation2d(2.75, 4), new Rotation2d(Math.toRadians(120)), new Rotation2d(0)),
                    new PathPoint(new Translation2d(0,4.5),  new Rotation2d(Math.toRadians(180)), new Rotation2d(0))));
                    */
    //start(ish) of driver vs auton
   /*  PathPlannerTrajectory trajectoryPlanned = PathPlanner.generatePath(new PathConstraints(2,2), List.of(
            new PathPoint(new Translation2d(0,0), new Rotation2d(0), new Rotation2d(0)),
            new PathPoint(new Translation2d(0,1.1938), new Rotation2d(Math.toRadians(90)), new Rotation2d(0)),
            new PathPoint(new Translation2d(-4.527, 1.1938), new Rotation2d(Math.toRadians(180)), new Rotation2d(0)),
            new PathPoint(new Translation2d(-4.527,3.9574), new Rotation2d(Math.toRadians(270)), new Rotation2d(0)),
            new PathPoint(new Translation2d(0,1.524), new Rotation2d(Math.toRadians(0)), new Rotation2d(0)),
            new PathPoint(new Translation2d(0,-0.889), new Rotation2d(Math.toRadians(270)), new Rotation2d(0))),
            new PathPoint(new Translation2d(-3.429,-0.889), new Rotation2d(Math.toRadians(180)), new Rotation2d(0)),
            new PathPoint(new Translation2d(-3.429, -3.429),new Rotation2d(Math.toRadians(270), new Rotation2d(0))),
            new PathPoint(new Translation2d())
    );
    */

/*        try {
      trajectory3 = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("pathplanner/Forward 3 Meters and 90 degrees.path"));
    }
    catch (IOException e) {
      DriverStation.reportError("Couldn't read file", false);
    }*/
    // System.out.println(trajectory2.toString());
    /*  Trajectory trajectory = TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(0,0,new Rotation2d(0)),
            new Pose2d(1.5, 0.1, new Rotation2d(Math.toRadians(30))), new Pose2d(3, 1, new Rotation2d(Math.toRadians(90))),
            new Pose2d(2.3, 1.5, new Rotation2d(Math.toRadians(120))), new Pose2d(1, 2, new Rotation2d(Math.toRadians(180))),
            new Pose2d(1.25, 2.2, new Rotation2d(Math.toRadians(0))), new Pose2d(2.25, 2.5, new Rotation2d(Math.toRadians(30))),
            new Pose2d(3, 3.25, new Rotation2d(Math.toRadians(90))), new Pose2d(2.75, 4, new Rotation2d(Math.toRadians(120))),
            new Pose2d(0,4.5, new Rotation2d(Math.toRadians(180)))), trajectoryConfig); */
    PIDController xController = new PIDController(Constants.kPXController, Constants.kIXController,Constants.kDXController);
    PIDController yController = new PIDController(Constants.kPYController, Constants.kIYController ,Constants.kDYController);
    HolonomicDriveController holonomicDriveController = new HolonomicDriveController(xController, yController, new ProfiledPIDController(Constants.kPThetaController, Constants.kIThetaController, 0, Constants.kThetaControllerConstraints));

    KugelControllerCommand swerveControllerCommand = new KugelControllerCommand(
            trajectoryPlanned,
            m_chassis::getPose2d,
            m_chassis.getKinematics(),
            holonomicDriveController,
            /*() -> {return trajectoryPlanned.getEndState().holonomicRotation;},*/
            m_chassis::setModuleStates,
            m_chassis);

    auton_command = new SequentialCommandGroup(new InstantCommand(()->m_chassis.resetOdometry(trajectoryPlanned.getInitialPose())),
            swerveControllerCommand, new InstantCommand(m_chassis::stopModules));

    return auton_command;
  }

}