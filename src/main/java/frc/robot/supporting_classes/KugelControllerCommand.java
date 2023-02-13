package frc.robot.supporting_classes;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

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

    public PathPlannerTrajectory.PathPlannerState sample(double time) {
        if (time <= m_trajectory.getInitialState().timeSeconds) return m_trajectory.getInitialState();
        if (time >= m_trajectory.getTotalTimeSeconds()) return m_trajectory.getEndState();

        int low = 1;
        int high = m_trajectory.getStates().size() - 1;

        // bin search tracjectory top sample
        while (low != high) {
          int mid = (low + high) / 2;
          if (m_trajectory.getState(mid).timeSeconds < time) {
            low = mid + 1;
          } else {
            high = mid;
          }
        }

        return m_trajectory.getState(low);
    }

    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    public void execute() {
        double time = m_timer.get();
        Trajectory.State desiredState = m_trajectory.sample(time);

        ChassisSpeeds targetChassisSpeeds = m_controller.calculate(m_pose.get(), desiredState, sample(time).holonomicRotation);
        SwerveModuleState[] targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

        m_outputModuleStates.accept(targetModuleStates);
    }


    public boolean isFinished() {
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }
}
