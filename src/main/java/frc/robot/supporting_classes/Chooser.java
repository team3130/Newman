package frc.robot.supporting_classes;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Function;

public class Chooser {
    private final SendableChooser<AutonCommand> m_autonChooser;
    private final RobotContainer container;
    private final TrajectoryConfig config;

    public Chassis m_chassis = new Chassis();


    public Chooser(SendableChooser<AutonCommand> m_autonChooser, RobotContainer container){
        this.m_autonChooser = m_autonChooser;
        this.container = container;

        Chassis chassis = container.getChassis();

        config = new TrajectoryConfig(Constants.kPhysicalMaxSpeedMetersPerSecond / 4, Constants.kMaxAccelerationDrive / 8)
                .setKinematics(chassis.getKinematics());
    }

    public AutonCommand autonGenerator(PathPlannerTrajectory trajectory) {
        PIDController xController = new PIDController(Constants.kPXController, Constants.kIXController,Constants.kDXController);
        PIDController yController = new PIDController(Constants.kPYController, Constants.kIYController ,Constants.kDYController);
        HolonomicDriveController holonomicDriveController = new HolonomicDriveController(xController, yController, new ProfiledPIDController(Constants.kPThetaController, Constants.kIThetaController, 0, Constants.kThetaControllerConstraints));

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                m_chassis::getPose2d,
                m_chassis.getKinematics(),
                holonomicDriveController,
                () -> {return trajectory.getEndState().holonomicRotation;},
                m_chassis::setModuleStates,
                m_chassis);


        AutonCommand cmd = new AutonCommand(swerveControllerCommand, trajectory.getInitialPose(), trajectory.getEndState().poseMeters);
        return cmd;
    }


}
