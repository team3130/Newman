package frc.robot.supportingClasses.Auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Newman_Constants.Constants;
import frc.robot.commands.Balance.RileyPark;
import frc.robot.commands.Balance.SearchBalance;
import frc.robot.commands.Intake.ToggleIntake;
import frc.robot.commands.Manipulator.ToggleManipulator;
import frc.robot.commands.Placement.AutoZeroExtensionArm;
import frc.robot.commands.Placement.AutoZeroRotryArm;
import frc.robot.commands.Placement.presets.GoToHighScoring;
import frc.robot.commands.TimedCommand;
import frc.robot.subsystems.*;
import org.opencv.core.Mat;

import java.nio.file.Path;
import java.util.List;

/**
 * A class to generate our auton paths from PathPlanner
 */
public class AutonManager {

    /**
     * The sendable chooser that gets put on shuffleboard.
     * It is a dropdown menu that can be used for selecting paths
     */
    private final SendableChooser<CommandBase> m_autonChooser;

    /**
     * The chassis subsystem
     */
    protected final Chassis m_chassis;

    /**
     * safe speeds for testing.
     * 2 m/s and 2 m/s/s for max velocity and max acceleration respectively
     */
    protected final PathConstraints safe_constraints;

    /**
     * wild speeds for if we want to go brrrrrrrrr
     */
    protected final PathConstraints violent_constraints;

    /**
     * really slow velocity for when we try to balance 
     */
    protected final PathConstraints balance_constraints;

    /**
     * The alliance that we are on so we don't have to spam networktables
     */
    private final DriverStation.Alliance alliance;

    /**
     * The intake subsystem
     */
    protected final Intake m_intake;

    /**
     * The rotary arm subsystem
     */
    protected final RotaryArm rotary;

    /**
     * The extension arm subsystem
     */
    protected final ExtensionArm extension;

    /**
     * The manipulator subsystem
     */
    protected final Manipulator m_manipulator;

    /**
     * Makes an object to make and manage auton paths.
     * The passed in subsystems are used for making AutonCommands that run routines in auton
     * Also calls {@link #populateChooser()}
     * @param chassis needs chassis so that commands made in here can use it
     * @param intake the intake mechanism
     * @param rotary the Rotary arm mechanism
     * @param extension the extension arm subsystem
     * @param manipulator the manipulator subsystem
     */
    public AutonManager(Chassis chassis, Intake intake, RotaryArm rotary, ExtensionArm extension, Manipulator manipulator) {
        this.m_autonChooser = new SendableChooser<>();
        this.m_chassis = chassis;

        safe_constraints = new PathConstraints(2, 2);
        violent_constraints = new PathConstraints(Constants.kPhysicalMaxSpeedMetersPerSecond, 3);
        balance_constraints = new PathConstraints(0.75, 2);

        Shuffleboard.getTab("Comp").add(m_autonChooser);

        m_intake = intake;
        this.rotary = rotary;
        this.extension = extension;
        m_manipulator = manipulator;

        alliance = DriverStation.getAlliance();

        populateChooser();
    }

    /**
     * Method to populate chooser with commands to follow.
     */
    private void populateChooser() {
        m_autonChooser.setDefaultOption("do nothing", new InstantCommand());

        // the string is the name passed into shuffleboard and the method call is to generate the method you will use
        m_autonChooser.addOption("move out of start intake pushy", makeCmdToIntakeAndGoForward()); // intake and spit out but then also move
        m_autonChooser.addOption("Intake spit", actuateIntake()); // intake and spit out
        m_autonChooser.addOption("place in auton move out", placeInAutonCone()); // place in auton and then don't leave the zone (good for if middle)
        m_autonChooser.addOption("place in auton don't move", placeInAuton()); // place in auton and move out. PathPoint so reliable and can start from anywhere
        // m_autonChooser.addOption("place in auton balance", placeInAutonBalance());
        m_autonChooser.addOption("pull out", generatePullOut()); // as the name suggests its the safest option
        // m_autonChooser.addOption("2 meters forward", generateExamplePathFromPoses());

        // m_autonChooser.addOption("place high", placeCubeHigh());
        m_autonChooser.addOption("dumb dumb balance", generateDumbBalance());
        m_autonChooser.addOption("place in auton move out and return", placeInAutonConeReturn());

        
        // m_autonChooser.addOption("marker path 2 cones HP", placeConeHighPlaceCubeHigh()); // really needs to be fixed. markers don't do anything right now yay
        // m_autonChooser.addOption("marker path 2 cones non-hp", loadTrajectory("place two cones non hp", true));
    }

    /**
     * what to command is currently selected on shuffleboard.
     * @return the command that is selected on shuffleboard
     */
    public CommandBase pick() {
        return m_autonChooser.getSelected();
    }

    /**
     * Generates an AutonCommand object from a {@link PathPlannerTrajectory} trajectory.
     * Trajectory's can be loaded from path planner with
     *  PathPlanner.loadPath(nameOfYourTrajectory, accelerationAndVelocityConstraints). For an example
     *  see {@link #generateExamplePathFromFile()}.
     * Trajectory's can be made from a list of points by passing in a list of positions:
     *   The first {@link edu.wpi.first.math.geometry.Translation2d} is a translation that the bot should
     *   follow
     *  The second parameter is a {@link Rotation2d} rotation,
     *      the rotation dictates the {@link edu.wpi.first.math.spline.Spline} that is made,
     *      which is basically the rotation of the plane that intersects the wheels when the bot gets to a position
     *      @see <a href="https://docs.google.com/presentation/d/1Us-ONi37lHcfJIlmSMmEISfco7uoSHt6dLa4VgWFYe8/edit?usp=sharing">
     *          An explanation on holonomic rotation vs heading.
     *          </a>
     *  The third parameter is another {@link Rotation2d} rotation,
     *      this rotation does not affect the path that the bot takes and instead is a holonomic rotation,
     *      that dictates the direction that the chassis will face.
     * @param trajectory the trajectory from PathPlanner to follow
     * @param useAprilTags whether or not to use april tags to update odometry during the path
     * @return an {@link AutonCommand} object that contains an auton path to run as well as the start and end points
     */
    public AutonCommand autonCommandGenerator(PathPlannerTrajectory trajectory, boolean useAprilTags) {
        return new AutonCommand(holonomicControllerGenerator(trajectory), trajectory, m_chassis, useAprilTags);
    }

    /**
     * Generates a holonomic controller command. Giving it all the necessary data from chassis and the PID controllers.
     * @param trajectory the trajectory that will be given to the holonomicControllerCommand
     * @return the generated Holonomic Controller Command
     */
    public HolonomicControllerCommand holonomicControllerGenerator(PathPlannerTrajectory trajectory) {
        PIDController xController = new PIDController(Constants.kPXController, Constants.kIXController,Constants.kDXController);
        PIDController yController = new PIDController(Constants.kPYController, Constants.kIYController ,Constants.kDYController);
        HolonomicDriveController holonomicDriveController = new HolonomicDriveController(xController, yController, new ProfiledPIDController(Constants.kPThetaController, Constants.kIThetaController, Constants.kDThetaController, Constants.kThetaControllerConstraints));

        return new HolonomicControllerCommand(
                trajectory,
                m_chassis::getPose2d,
                m_chassis.getKinematics(),
                holonomicDriveController,
                m_chassis::setModuleStates,
                m_chassis);
    }

    /**
     * Generates an auton command which requires placement subsystems calling:
     *  {@link #holonomicControllerGenerator(PathPlannerTrajectory)}.
     * The passed in subsystems are all stored in the {@link AutonManager} object.
     * 
     * @param trajectory the trajectory from PathPlanner to follow
     * @param useAprilTags whether or not to use april tags to update odometry during the path
     * @return
     */
    public AutonCommand autonCommandGeneratorPlacement(PathPlannerTrajectory trajectory, boolean useAprilTags) {
        return new AutonCommand(holonomicControllerGenerator(trajectory), trajectory, m_chassis, rotary, extension, m_manipulator, useAprilTags);
    }

    /**
     * Wraps the command with a call to reset odometry before running the first command
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
                    new InstantCommand(() -> m_chassis.resetOdometry(new Pose2d(command.getStartPosition().getTranslation(), command.getStartRotation()))),
                    command, new InstantCommand(m_chassis::stopModules)
        );
    }

    /**
     * A method to help with on the fly trajectory generation. 
     * As of current it will still update position with april tags however this may need to be changed for better effect in the future.
     * 
     * @param current the current position of the bot / start position of the trajectory
     * @param endPoint the end position of the bot / end position of the trajectory
     * @return the Auton command without placement support that will follow the path.
     */
    public AutonCommand onTheFlyGenerator(Pose2d current, Pose2d endPoint) {
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(violent_constraints,
                new PathPoint(current.getTranslation(), new Rotation2d(), current.getRotation()),
                new PathPoint(endPoint.getTranslation(), new Rotation2d(), endPoint.getRotation())
        );

        //TODO: if results are undesired change the useAprilTags to false
        return autonCommandGenerator(trajectory, true);
    }

    /**
     * loads a PathPlanner trajectory with optional marker support.
     * If you don't want marker support then this command won't require placement subsystems.
     * Any auton commands that are made using this command will use april tags.
     * 
     * @param nameOfFile the name of the path in PathPlanner
     * @param requirePlacement whether we should require placement or not
     * @return the generated auton command which requires placement and uses april tags
     */
    public AutonCommand loadTrajectory(String nameOfFile, boolean requirePlacement) {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(nameOfFile, safe_constraints); //TODO: Make this violent constraints
        PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance());
        return (requirePlacement) ? autonCommandGeneratorPlacement(trajectory, true) : autonCommandGenerator(trajectory, true);
    }

    /**
     * Example for how to generate a trajectory and generate a path from a list of {@link PathPoint} object
     * @see <a href="https://docs.google.com/document/d/1RInEhl8mW1UKMP4AbvWWiWmfI4klbDfyZLJbw1zbjDo/edit#heading=h.lie7pmqbolmu">
     *     Explanation of PathPoint objects
     *     </a>
     * @return the command generated
     */
    public CommandBase generateExamplePathFromPoses() {
        // the trajectory being made
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                /* Max velocity and acceleration the path will follow along the trapezoid profile */
                safe_constraints,
                /* Each path point is a 2 poses and 2 rotations see explanation here:
                https://docs.google.com/document/d/1RInEhl8mW1UKMP4AbvWWiWmfI4klbDfyZLJbw1zbjDo/edit#heading=h.lie7pmqbolmu */
                new PathPoint(
                        new Translation2d(0, 0),
                        new Rotation2d(), new Rotation2d()),
                new PathPoint(new Translation2d(2, 0), new Rotation2d(), new Rotation2d(0))
        );

        return autonCommandGenerator(trajectory, false);
    }

    /**
     * This example trajectory is a question mark.
     * @return the Question mark command
     */
    public Command generateExamplePathFromFile() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("Question Mark", safe_constraints);
        PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, alliance);
        AutonCommand autonCommand = autonCommandGenerator(trajectory, false);
        return wrapCmd(autonCommand);
    }

    /**
     * on the-fly generates a path to go back to origin
     * @param Current the current position of the robot or the position of the robot when we plan on running the command
     * @return the generated path as an Auton command
     */
    public CommandBase backToStart(Pose2d Current) {
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                violent_constraints,

                new PathPoint(
                        Current.getTranslation(),
                        new Rotation2d(0), m_chassis.getRotation2d()
                ),

                new PathPoint(new Translation2d(0, 0), new Rotation2d(), new Rotation2d())
        );

        return autonCommandGenerator(trajectory, true);
    }

    /**
     * on-the-fly generates a path to go to the closest location where you can place a game element from
     * @param current the current robots position or the position of the robot when the path will start
     * @return the generated path as an auton command
     */
    public AutonCommand makeCmdToGoToPlace(Pose2d current) {
        final int index = (int) (current.getY() * 2.5);
        final double y_value = ((Constants.Field.yPositionsForRowBounds[index] - (Constants.Field.yPositionsForRowBounds[index + 1]) / 2)) + Constants.Field.yPositionsForRowBounds[index];
        final double x_value = (DriverStation.getAlliance() == DriverStation.Alliance.Blue) ? Constants.Field.xPositionForGridBlue : Constants.Field.xPositionForGridRed;
        final double rotation = (DriverStation.getAlliance() == DriverStation.Alliance.Blue) ? Math.PI : 0;
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(safe_constraints,
                new PathPoint(
                        current.getTranslation(),
                        new Rotation2d(0), m_chassis.getRotation2d()
                ),

                new PathPoint(new Translation2d(x_value, y_value), new Rotation2d(), new Rotation2d(rotation)));

        return autonCommandGenerator(trajectory, true);
    }

    /**
     * On the fly trajectory generation to go to the human player station
     * @param current the current position on the field
     * @return the generated path in a wrapped command
     */
    public CommandBase makeCmdToGoToHumanPlayerStation(Pose2d current) {
        final double x_value;
        final double y_value;
        final double rotation;

        x_value = Constants.Field.xPositionForBlueHumanPlayerStation;
        y_value = Constants.Field.yPositionForBlueHumanPlayerStation;
        rotation = Constants.Field.rotationForBlueHumanPlayerStation;

        PathPlannerTrajectory trajectory = PathPlanner.generatePath(safe_constraints,
                new PathPoint(current.getTranslation(), new Rotation2d(0), current.getRotation()),
                new PathPoint(new Translation2d(x_value, y_value), new Rotation2d(), new Rotation2d(rotation)));

        trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance());

        return autonCommandGenerator(trajectory, true);
    }

    /**
     * Makes a command that toggles the intake.
     * @return A command that starts at 0, 0 and actuates intake
     */
    public CommandBase actuateIntake() {
        return new ToggleIntake(m_intake);
    }


    /**
     * Makes a command to actuate intake and go forward
     * @return command for intaking and going forward
     */
    public CommandBase makeCmdToIntakeAndGoForward() {
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                violent_constraints,
                new PathPoint(new Translation2d(0, 0),
                        new Rotation2d(), new Rotation2d(0)
                ),

                new PathPoint(new Translation2d(1.25, 0), new Rotation2d(), new Rotation2d(0))
        );

        AutonCommand command = autonCommandGenerator(trajectory, false);
        return new SequentialCommandGroup(new ToggleIntake(m_intake), wrapCmd(command));
    }

    /**
     * Requires odometry from april tags to be off in auton and for the traajectory to not be transformed by alliance
     * @return A command to place in auton
     */
    public SequentialCommandGroup placeInAuton() {
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                safe_constraints,
                new PathPoint(
                        new Translation2d(0, 0),
                        new Rotation2d(0), new Rotation2d()
                ),

                new PathPoint(new Translation2d(0.75, 0), new Rotation2d(0), new Rotation2d(0))
        );

        PathPlannerTrajectory trajectory2 = PathPlanner.generatePath(
                safe_constraints,
                new PathPoint(
                        new Translation2d(0.75, 0),
                        new Rotation2d(0), new Rotation2d()
                ),

                new PathPoint(new Translation2d(0.25, 0), new Rotation2d(0), new Rotation2d(0))
        );

        AutonCommand command = autonCommandGenerator(trajectory, false);
        AutonCommand command2 = autonCommandGenerator(trajectory2, false);
        return new SequentialCommandGroup(
                    new ToggleManipulator(m_manipulator),
                    new GoToHighScoring(rotary, extension),
                    wrapCmd(command),
                    new ToggleManipulator(m_manipulator),
                    wrapCmd(command2),
                    new AutoZeroExtensionArm(extension),
                    new AutoZeroRotryArm(rotary));
    }

    /**
     * Requires odometry from april tags to be off in auton and for the traajectory to not be transformed by alliance
     * @return A command to place in auton
     */
    public SequentialCommandGroup placeInAutonBalance() {
        return placeInAuton().andThen(new SequentialCommandGroup(new SearchBalance(m_chassis), new RileyPark(m_chassis)));
    }

    /**
     * Place a cone in auton.
     * Requires odometry from april tags to be off in auton and for the traajectory to not be transformed by alliance.
     * @return a command to place a cone in auton.
     */
    public CommandBase placeInAutonCone() {
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                safe_constraints,
                new PathPoint(
                        new Translation2d(0, 0),
                        new Rotation2d(0), new Rotation2d()
                ),

                new PathPoint(new Translation2d(0.75, 0), new Rotation2d(0), new Rotation2d(0))
        );

        PathPlannerTrajectory trajectory2 = PathPlanner.generatePath(
                safe_constraints,
                new PathPoint(
                        new Translation2d(0.75, 0),
                        new Rotation2d(0), new Rotation2d()
                ),

                new PathPoint(new Translation2d(-3, 0), new Rotation2d(0), new Rotation2d(0)),
                new PathPoint(new Translation2d(-4, 0), new Rotation2d(0), new Rotation2d(Math.PI))
        );

        AutonCommand command = autonCommandGenerator(trajectory, false);
        AutonCommand command2 = autonCommandGenerator(trajectory2, false);

        return
            new SequentialCommandGroup(
                new AutoZeroRotryArm(rotary),
                new AutoZeroExtensionArm(extension),
                new ToggleManipulator(m_manipulator),
                new TimedCommand(0.2),
                new GoToHighScoring(rotary, extension),
                command,
                new ToggleManipulator(m_manipulator),
                new TimedCommand(0.2),
                new ParallelCommandGroup(
                        command2,
                        new SequentialCommandGroup(
                                new AutoZeroExtensionArm(extension),
                                new AutoZeroRotryArm(rotary)
                        )
                ));
    }

    /**
     * Place a cone in auton.
     * Requires odometry from april tags to be off in auton and for the traajectory to not be transformed by alliance.
     * @return a command to place a cone in auton.
     */
    public CommandBase placeInAutonConeReturn() {
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                safe_constraints,
                new PathPoint(
                        new Translation2d(0, 0),
                        new Rotation2d(0), new Rotation2d()
                ),

                new PathPoint(new Translation2d(0.75, 0), new Rotation2d(0), new Rotation2d(0))
        );

        PathPlannerTrajectory trajectory2 = PathPlanner.generatePath(
                safe_constraints,
                new PathPoint(
                        new Translation2d(0.75, 0),
                        new Rotation2d(0), new Rotation2d()
                ),

                new PathPoint(new Translation2d(-4, 0), new Rotation2d(0), new Rotation2d(0)),
                new PathPoint(new Translation2d(-1, 0), new Rotation2d(), new Rotation2d())
        );

        AutonCommand command = autonCommandGenerator(trajectory, false);
        AutonCommand command2 = autonCommandGenerator(trajectory2, false);

        return
                new SequentialCommandGroup(
                        new AutoZeroRotryArm(rotary),
                        new AutoZeroExtensionArm(extension),
                        new ToggleManipulator(m_manipulator),
                        new TimedCommand(0.2),
                        new GoToHighScoring(rotary, extension),
                        command,
                        new ToggleManipulator(m_manipulator),
                        new TimedCommand(0.2),
                        new ParallelCommandGroup(
                                command2,
                                new SequentialCommandGroup(
                                        new AutoZeroExtensionArm(extension),
                                        new AutoZeroRotryArm(rotary)
                                )
                        ));
    }

    /**
     * Place a cone in high at start and then place a cube in high.
     * @return the auton command for the generated trajectory wrapped
     */
    public CommandBase placeConeHighPlaceCubeHigh() {
        PathPlannerTrajectory trajectoryHP = PathPlanner.loadPath("place two cones high hp", new PathConstraints(1.5, 1.5));
        return autonCommandGeneratorPlacement(trajectoryHP, true);

    /*  PathPlannerTrajectory trajectorynonHP = PathPlanner.loadPath(, safe_constraints);
        AutonCommand commandnonHP = autonCommandGeneratorPlacement(trajectorynonHP);*/
    }

    /**
     * He-he.
     * Generates a trajectory to leave the community zone by going 1.5 meters forwards.
     * @return the generated trajectory in a wrapped command.
     */
    public CommandBase generatePullOut() {
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(safe_constraints, 
        new PathPoint(
            new Translation2d(0, 0),
            new Rotation2d(0), new Rotation2d()
        ),

        new PathPoint(new Translation2d(1.5, 0), new Rotation2d(0), new Rotation2d())
        );

        AutonCommand command = autonCommandGenerator(trajectory, false);
        return wrapCmd(command);
    }

    /**
     * Generates a trajectory to go to the start of the main path. 
     * Is ran before the main path in auton to make sure that we start in the corrent spot.
     * Basically a wrapper for the main auton command to make sure that we start in the correct spot.
     * @param mainPath the main command that will be ran in auton
     * @return the command group for going to the start of the path and then running the passed in path.
     */
    public SequentialCommandGroup goToStartOfCommand(AutonCommand mainPath) {
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
            safe_constraints,
            new PathPoint(m_chassis.getPose2d().getTranslation(), new Rotation2d(), m_chassis.getRotation2d()), 
            new PathPoint(mainPath.getStartPosition().getTranslation(), new Rotation2d(), mainPath.getStartRotation()));
        AutonCommand goToStart = autonCommandGenerator(trajectory, true);

        return new SequentialCommandGroup(goToStart, mainPath);
    }

    public AutonCommand placeCubeHigh() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("place cube high non hp", safe_constraints);
        return autonCommandGenerator(trajectory, true);
    }

    public CommandBase setpointBalance(int dir) {
        // the trajectory being made
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                /* Max velocity and acceleration the path will follow along the trapezoid profile */
                balance_constraints,
              
                new PathPoint(
                        m_chassis.getPose2d().getTranslation(),
                        new Rotation2d(), new Rotation2d()),
                new PathPoint(new Translation2d(m_chassis.getPose2d().getTranslation().getX() + dir*Constants.Balance.toStationCenterDistance, m_chassis.getPose2d().getTranslation().getY()), new Rotation2d(), new Rotation2d(0))
        );

        

        AutonCommand command = autonCommandGenerator(trajectory,  false);
        return wrapCmd(command);
    }

    public CommandBase generateDumbBalance() {
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                /* Max velocity and acceleration the path will follow along the trapezoid profile */
                balance_constraints,

                List.of(new PathPoint(new Translation2d(0.25, 0),
                        new Rotation2d(), new Rotation2d()),
                new PathPoint(new Translation2d(-1.95, 0), new Rotation2d(), new Rotation2d(0)),
                        new PathPoint(new Translation2d(-3.5, 0), new Rotation2d(), new Rotation2d()),
                        new PathPoint(new Translation2d(-1.35, 0), new Rotation2d(), new Rotation2d()))
        );

        PathPlannerTrajectory trajectory1 = PathPlanner.generatePath(
                safe_constraints,
                new PathPoint(
                        new Translation2d(0, 0),
                        new Rotation2d(0), new Rotation2d()
                ),

                new PathPoint(new Translation2d(0.75, 0), new Rotation2d(0), new Rotation2d(0))
        );

        PathPlannerTrajectory trajectory2 = PathPlanner.generatePath(
                safe_constraints,
                new PathPoint(
                        new Translation2d(0.75, 0),
                        new Rotation2d(0), new Rotation2d()
                ),

                new PathPoint(new Translation2d(0.25, 0), new Rotation2d(0), new Rotation2d(0))
        );

        AutonCommand command = autonCommandGenerator(trajectory1, false);
        AutonCommand command2 = autonCommandGenerator(trajectory2, false);
        AutonCommand command3 = autonCommandGenerator(trajectory, false);
        return new SequentialCommandGroup(
                new ToggleManipulator(m_manipulator),
                new GoToHighScoring(rotary, extension),
                wrapCmd(command),
                new ToggleManipulator(m_manipulator),
                wrapCmd(command2),
                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                        new AutoZeroExtensionArm(extension),
                        new AutoZeroRotryArm(rotary))),
                    command3);
    }


}