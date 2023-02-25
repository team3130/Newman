package frc.robot.supportingClasses;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Newman_Constants.Constants;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class KugelControllerCommand extends CommandBase {
  protected final Timer m_timer = new Timer();
  protected final PathPlannerTrajectory m_trajectory;
  protected final Supplier<Pose2d> m_pose;
  protected final SwerveDriveKinematics m_kinematics;
  protected final HolonomicDriveController m_controller;
  protected final Consumer<SwerveModuleState[]> m_outputModuleStates;

    public KugelControllerCommand(PathPlannerTrajectory trajectory, Supplier<Pose2d> robotPose, SwerveDriveKinematics kinematics, HolonomicDriveController holonomicDriveController, Consumer<SwerveModuleState[]> states, Subsystem requirement) {
        m_trajectory = trajectory;
        m_pose = robotPose;
        m_kinematics = kinematics;
        m_controller = holonomicDriveController;
        m_outputModuleStates = states;
    }

    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    public void execute() {
        double time = m_timer.get();
        PathPlannerTrajectory.PathPlannerState desiredState = (PathPlannerTrajectory.PathPlannerState) m_trajectory.sample(time);

        ChassisSpeeds targetChassisSpeeds = m_controller.calculate(m_pose.get(), desiredState,
                desiredState.holonomicRotation);
        SwerveModuleState[] targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

        m_outputModuleStates.accept(targetModuleStates);
    }

    public boolean inRange(Pose2d pose1, Pose2d pose2) {
        return Math.abs(pose1.getX() - pose2.getX()) < 0.1 &&
                Math.abs(pose1.getY() - pose2.getY()) < 0.1 &&
                Math.abs(pose1.getRotation().getDegrees() - pose2.getRotation().getDegrees()) < 2.5;
    }

    public boolean isFinished() {
        boolean timeHasPassed = m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
        if (Constants.debugMode) {
            return inRange(m_pose.get(), m_trajectory.getEndState().poseMeters);
        }
        else {
            return timeHasPassed;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }
}
