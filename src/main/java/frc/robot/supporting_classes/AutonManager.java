package frc.robot.supporting_classes;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Newman_Constants.Constants;
import frc.robot.subsystems.Chassis;

public class AutonManager {
    private final SendableChooser<Command> m_autonChooser;
    protected Chassis m_chassis;

    protected PathConstraints safe_constraints;
    protected PathConstraints violent_constraints;

    /**
     * Makes an object to make and manage auton paths.
     * Also calls {@link #populateChooser()}
     * @param chassis needs chassis so that commands made in here can use it
     */
    public AutonManager(Chassis chassis){
        this.m_autonChooser = new SendableChooser<>();
        this.m_chassis = chassis;

        safe_constraints = new PathConstraints(2, 2);
        violent_constraints = new PathConstraints(4, 3);

        SmartDashboard.putData(m_autonChooser);

        populateChooser();
    }

    /**
     * Method to populate chooser with commands to follow.
     */
    private void populateChooser() {
        m_autonChooser.setDefaultOption("do nothing", generateExamplePathFromPoses());

        // the string is the name passed into shuffleboard and the method call is to generate the method you will use
        // m_autonChooser.addOption("3 Meter", generate3MeterDrive());
        // m_autonChooser.addOption("Question mark", generateExamplePathFromFile());
        // m_autonChooser.addOption("player side", generatepWeekZeroPath());
        // m_autonChooser.addOption("far side", generatepWeekZeroPath2());
        m_autonChooser.addOption("feelin spicy", generateExamplePathFromPoses());
    }

    /**
     * what to command is currently selected on shuffleboard.
     * @return the command that is selected on shuffleboard
     */
    public Command pick() {
        return m_autonChooser.getSelected();
    }

    /**
     * Generates an AutonCommand object from a {@link PathPlannerTrajectory} trajectory.
     * Trajectory's can be loaded from path planner with
     *  PathPlanner.loadPath(nameOfYourTrajectory, accelerationAndVelocityConstraints). For an example
     *  see {@link #generate3MeterDrive()}.
     * Trajectory's can be made from a list of points by passing in a list of positions:
     *   The first {@link edu.wpi.first.math.geometry.Translation2d} is a translation that the bot should
     *   follow
     *  The second parameter is a {@link Rotation2d} rotation,
     *      the rotation dictates the {@link edu.wpi.first.math.spline.Spline} that is made,
     *      which is basically the rotation of the plane that intersects the wheels when the bot gets to a position
     *      <a href=https://docs.google.com/presentation/d/1Us-ONi37lHcfJIlmSMmEISfco7uoSHt6dLa4VgWFYe8/edit?usp=sharing>
     *          An explanation on holonomic rotation vs heading.
     *          </a>
     *  The third parameter is another {@link Rotation2d} rotation,
     *      this rotation does not affect the path that the bot takes and instead is a holonomic rotation,
     *      that dictates the direction that the chassis will face.
     * @param trajectory the trajectory from PathPlanner to follow
     * @return an {@link AutonCommand} object that contains an auton path to run as well as the start and end points
     */
    public AutonCommand autonCommandGenerator(PathPlannerTrajectory trajectory) {
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


        return new AutonCommand(swerveControllerCommand, trajectory.getInitialPose(), trajectory.getEndState().poseMeters);
    }

    /**
     * Wraps the command with a call to reset odometry before running the {@param first} command
     * Then adds all the commands passed into restOfCommands
     * Finally wraps the end with a call to stop the swerve modules
     * @param first the first command that you run with a pose for where you start
     * @param restOfCommands all the commands you want to run after the first one
     * @return the final command
     */
    public SequentialCommandGroup wrapCmd(AutonCommand first, Command... restOfCommands) {
        SequentialCommandGroup commandgroup = new SequentialCommandGroup(
                new InstantCommand(() -> m_chassis.resetOdometry(first.getStartPosition())),
                first.getCmd()
        );
        // add the rest of the commands passed in
        commandgroup.addCommands(restOfCommands);
        // stop the modules
        commandgroup.addCommands(new InstantCommand(m_chassis::stopModules));

        return commandgroup;
    }

    /**
     * Wraps the command with calls to reset odometery before running the command
     * Stops the modules when done
     * Only run if you are only following one path or only have one command
     * Overload of {@link #wrapCmd(AutonCommand, Command...)}
     * @param command first command to run in the group, should contain a start position
     * @return the final routine
     */
    public SequentialCommandGroup wrapCmd(AutonCommand command) {
                return new SequentialCommandGroup(
                new InstantCommand(() -> m_chassis.resetOdometry(command.getStartPosition())),
                command.getCmd(),
                new InstantCommand(m_chassis::stopModules)
        );
    }

    /**
     * Generates a path to drive forward three meters and face 90 degrees clockwise
     * @return a sequentialCommandGroup to run on auton init
     */
    public Command generate3MeterDrive() {
        // load the trajectory from the filesystem
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("3meter", safe_constraints);

        // store the autonCommand that gets generated from the trajectory
        AutonCommand autoCommand = autonCommandGenerator(trajectory);

        // wrap the command to reset odometry on start and stop motors on end
        return wrapCmd(autoCommand);
    }

    /**
     * Example for how to generate a trajectory and generate a path from a list of {@link PathPoint} object
     * <a href=https://docs.google.com/document/d/1RInEhl8mW1UKMP4AbvWWiWmfI4klbDfyZLJbw1zbjDo/edit#heading=h.lie7pmqbolmu>
     *     Explanation of PathPoint objects</a>
     * @return the command generated
     */
    public Command generateExamplePathFromPoses() {
        // the trajectory being made
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                /* Max velocity and acceleration the path will follow along the trapezoid profile */
                safe_constraints,
                /* Each path point is a 2 poses and 2 rotations see explanation here:
                https://docs.google.com/document/d/1RInEhl8mW1UKMP4AbvWWiWmfI4klbDfyZLJbw1zbjDo/edit#heading=h.lie7pmqbolmu */
                new PathPoint(
                        new Translation2d(0, 0),
                        new Rotation2d(), new Rotation2d()),
                new PathPoint(new Translation2d(2, 0), new Rotation2d(), new Rotation2d(Math.toRadians(90)))
        );

        AutonCommand command = autonCommandGenerator(trajectory);
        return wrapCmd(command);
    }

    /**
     * This example trajectory is a question mark
     * @return the Question mark command
     */
    public Command generateExamplePathFromFile() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("Question Mark", safe_constraints);
        AutonCommand autonCommand = autonCommandGenerator(trajectory);
        return wrapCmd(autonCommand);
    }

    public Command generatepWeekZeroPath() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("week zero human player path", safe_constraints);
        AutonCommand autonCommand = autonCommandGenerator(trajectory);
        return wrapCmd(autonCommand);
    }

    public Command generatepWeekZeroPath2() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("week zero path 2 farside", safe_constraints);
        AutonCommand autonCommand = autonCommandGenerator(trajectory);
        return wrapCmd(autonCommand);
    }

}
