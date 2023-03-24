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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Newman_Constants.Constants;
import frc.robot.commands.Chassis.presets.DriveForwardAndIntake;
import frc.robot.commands.Chassis.presets.GoToClampAndDriveOut;
import frc.robot.commands.Intake.ToggleIntake;
import frc.robot.commands.Manipulator.ToggleGrabber;
import frc.robot.commands.Placement.AutoZeroExtensionArm;
import frc.robot.commands.Placement.presets.GoToHighScoring;
import frc.robot.subsystems.*;

/**
 * A class to generate our auton paths from PathPlanner
 */
public class AutonManager {
    private final SendableChooser<Command> m_autonChooser; // shuffleboard dropdown menu for selecting the path
    protected Chassis m_chassis; // the chassis object

    protected PathConstraints safe_constraints; // safe speeds for testing
    protected PathConstraints violent_constraints; // wild speeds for if we want to go brrrrrrrrr

    private final DriverStation.Alliance alliance;

    protected IntakePivot m_intake;
    protected RotaryArm rotary;
    protected ExtensionArm extension;
    protected Manipulator m_manipulator;

    /**
     * Makes an object to make and manage auton paths.
     * Also calls {@link #populateChooser()}
     * @param chassis needs chassis so that commands made in here can use it
     */
    public AutonManager(Chassis chassis, IntakePivot intake, RotaryArm rotary, ExtensionArm extension, Manipulator manipulator){
        this.m_autonChooser = new SendableChooser<>();
        this.m_chassis = chassis;

        safe_constraints = new PathConstraints(2, 2);
        violent_constraints = new PathConstraints(Constants.kPhysicalMaxSpeedMetersPerSecond, Constants.kPhysicalMaxSpeedMetersPerSecond);

        Shuffleboard.getTab("Comp").add(m_autonChooser);

        m_intake = intake;
        this.rotary = rotary;
        this.extension = extension;
        m_manipulator = manipulator;

        populateChooser();

        alliance = DriverStation.getAlliance();
    }

    /**
     * Method to populate chooser with commands to follow.
     */
    private void populateChooser() {
        m_autonChooser.setDefaultOption("do nothing", new InstantCommand());

        // the string is the name passed into shuffleboard and the method call is to generate the method you will use
        // m_autonChooser.addOption("3 Meter", generate3MeterDrive());
        // m_autonChooser.addOption("Question mark", generateExamplePathFromFile());
        // m_autonChooser.addOption("player side", generatepWeekZeroPath());
        // m_autonChooser.addOption("far side", generatepWeekZeroPath2());
        // m_autonChooser.addOption("feelin spicy", generateExamplePathFromPoses());
        // m_autonChooser.addOption("circuit", complexPathTest());
        // m_autonChooser.addOption("AprilTagTesting",aprilTagTesting());
        m_autonChooser.addOption("move out of start intake pushy", new DriveForwardAndIntake(m_chassis, m_intake, this));
        m_autonChooser.addOption("move out and clamp", new GoToClampAndDriveOut(m_chassis, m_manipulator, this));
        m_autonChooser.addOption("Two meter forward", generateExamplePathFromPoses()); // two meter forward (stable)
        m_autonChooser.addOption("Intake spit", actuateIntake());
        m_autonChooser.addOption("top dumb", generateTopDumb());
        m_autonChooser.addOption("bottom dumb", generateBottomDumb());
        m_autonChooser.addOption("mid placement start top", generateMidPlaceTopStart());
        if (Constants.debugMode) {
            m_autonChooser.addOption("marker path <- not for comp", generateMarkerPath());
        }
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
        return new AutonCommand(holonomicControllerGenerator(trajectory), trajectory, m_chassis);
    }

    public HolonomicControllerCommand holonomicControllerGenerator(PathPlannerTrajectory trajectory) {
        PIDController xController = new PIDController(Constants.kPXController, Constants.kIXController,Constants.kDXController);
        PIDController yController = new PIDController(Constants.kPYController, Constants.kIYController ,Constants.kDYController);
        HolonomicDriveController holonomicDriveController = new HolonomicDriveController(xController, yController, new ProfiledPIDController(Constants.kPThetaController, Constants.kIThetaController, 0, Constants.kThetaControllerConstraints));

        trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance());

        return new HolonomicControllerCommand(
                trajectory,
                m_chassis::getPose2d,
                m_chassis.getKinematics(),
                holonomicDriveController,
                m_chassis::setModuleStates,
                m_chassis);

    }

    public AutonCommand autonCommandGeneratorPlacement(PathPlannerTrajectory trajectory) {
        return new AutonCommand(holonomicControllerGenerator(trajectory), trajectory, m_chassis, rotary, extension, m_manipulator);
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
                    new InstantCommand(() -> m_chassis.resetOdometry(new Pose2d(command.getStartPosition().getTranslation(), command.getStartRotation()))),
                    command, new InstantCommand(m_chassis::stopModules)
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
                violent_constraints,
                /* Each path point is a 2 poses and 2 rotations see explanation here:
                https://docs.google.com/document/d/1RInEhl8mW1UKMP4AbvWWiWmfI4klbDfyZLJbw1zbjDo/edit#heading=h.lie7pmqbolmu */
                new PathPoint(
                        new Translation2d(0, 0),
                        new Rotation2d(), new Rotation2d()),
                new PathPoint(new Translation2d(2, 0), new Rotation2d(), new Rotation2d(0))
        );

        AutonCommand command = autonCommandGenerator(trajectory);
        return wrapCmd(command);
    }

    public Command aprilTagTesting(){
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(safe_constraints, new PathPoint(
                new Translation2d(0, 0), new Rotation2d(), new Rotation2d()),
                new PathPoint( new Translation2d(-3, 0),  new Rotation2d(), new Rotation2d()));

        AutonCommand command = autonCommandGenerator(trajectory);
        return wrapCmd(command);

    }


    public CommandBase complexPathTest() {
        PathPlannerTrajectory circuit = PathPlanner.loadPath("circuit", safe_constraints);
        PathPlannerTrajectory circuitLoop2 = PathPlanner.loadPath("circuit", safe_constraints);
        PathPlannerTrajectory circuitLoop3 = PathPlanner.loadPath("circuit", safe_constraints);
        circuit.concatenate(circuitLoop2);
        circuit.concatenate(circuitLoop3);

        return wrapCmd(autonCommandGenerator(circuit));
    }

    public CommandBase backToStart(Pose2d Current) {
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                violent_constraints,

                new PathPoint(
                        Current.getTranslation(),
                        new Rotation2d(0), m_chassis.getRotation2d()
                ),

                new PathPoint(new Translation2d(0, 0), new Rotation2d(), new Rotation2d())
        );

        AutonCommand commad = autonCommandGenerator(trajectory);
        return wrapCmd(commad);
    }

    /**
     * This example trajectory is a question mark
     * @return the Question mark command
     */
    public Command generateExamplePathFromFile() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("Question Mark", safe_constraints);
        PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, alliance);
        AutonCommand autonCommand = autonCommandGenerator(trajectory);
        return wrapCmd(autonCommand);
    }

    public Command generateWeekZeroPath() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("week zero human player path", safe_constraints);
        PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, alliance);
        AutonCommand autonCommand = autonCommandGenerator(trajectory);
        return wrapCmd(autonCommand);
    }

    public Command generateWeekZeroPath2() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("week zero path 2 farside", safe_constraints);
        PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, alliance);
        AutonCommand autonCommand = autonCommandGenerator(trajectory);
        return wrapCmd(autonCommand);
    }

    public CommandBase makeCmdToGoToPlace(Pose2d current) {
        final int index = (int) (current.getY() * 2.5);
        final double y_value = ((Constants.Field.yPositionsForRowBounds[index] - (Constants.Field.yPositionsForRowBounds[index + 1]) / 2)) + Constants.Field.yPositionsForRowBounds[index];
        final double x_value = (DriverStation.getAlliance() == DriverStation.Alliance.Blue) ? Constants.Field.xPositionForGridBlue : Constants.Field.xPositionForGridRed;
        final double rotation = (DriverStation.getAlliance() == DriverStation.Alliance.Blue) ? 0 : Math.PI;
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(safe_constraints,
                new PathPoint(
                        current.getTranslation(),
                        new Rotation2d(0), m_chassis.getRotation2d()
                ),

                new PathPoint(new Translation2d(x_value, y_value), new Rotation2d(), new Rotation2d(rotation)));

        AutonCommand command = autonCommandGenerator(trajectory);
        return wrapCmd(command);
    }

    public CommandBase makeCmdToGoToHumanPlayerStation(Pose2d current) {
        final double x_value;
        final double y_value;
        final double rotation;

        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            x_value = Constants.Field.xPositionForBlueHumanPlayerStation;
            y_value = Constants.Field.yPositionForBlueHumanPlayerStation;
            rotation = Constants.Field.rotationForBlueHumanPlayerStation;
        } else {
            x_value = Constants.Field.xPositionForRedHumanPlayerStation;
            y_value = Constants.Field.yPositionForRedHumanPlayerStation;
            rotation = Constants.Field.rotationForRedHumanPlayerStation;
        }

        PathPlannerTrajectory trajectory = PathPlanner.generatePath(safe_constraints,
                new PathPoint(current.getTranslation(), new Rotation2d(0), current.getRotation()),
                new PathPoint(new Translation2d(x_value, y_value), new Rotation2d(), new Rotation2d(rotation)));

        AutonCommand command = autonCommandGenerator(trajectory);
        return wrapCmd(command);
    }

    private CommandBase generateMovOutOfStart() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("MoveOutOfStart", safe_constraints);
        CommandBase command = wrapCmd(autonCommandGenerator(trajectory));
        return new SequentialCommandGroup(new ToggleIntake(m_intake), command);
    }

    private CommandBase generateMoveOutAndClamp() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("clamp and move out", safe_constraints);
        CommandBase command = wrapCmd(autonCommandGenerator(trajectory));
        return new SequentialCommandGroup(new AutoZeroExtensionArm(extension), new ToggleGrabber(m_manipulator), command);
    }

    public CommandBase actuateIntake() {
        return new ToggleIntake(m_intake);
    }

    public CommandBase generateTopDumb() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("dumb leave top", safe_constraints);
        CommandBase command = wrapCmd(autonCommandGenerator(trajectory));
        return new SequentialCommandGroup(new ToggleGrabber(m_manipulator), command, new GoToHighScoring(rotary, extension));
    }

    public CommandBase generateBottomDumb() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("dumb leave bottom", safe_constraints);
        CommandBase command = wrapCmd(autonCommandGenerator(trajectory));
        return new SequentialCommandGroup(new ToggleGrabber(m_manipulator), command, new GoToHighScoring(rotary, extension));
    }

    public CommandBase generateMarkerPath() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("marker test", safe_constraints);
        AutonCommand command = autonCommandGeneratorPlacement(trajectory);
        return wrapCmd(command);
    }


    public CommandBase generateMidPlaceTopStart() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("place mid top", safe_constraints);
        AutonCommand command = autonCommandGeneratorPlacement(trajectory);
        return wrapCmd(command);
    }

    public CommandBase makeCmdToIntakeAndGoForward(Pose2d current) {
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                violent_constraints,
                new PathPoint(
                        current.getTranslation(),
                        new Rotation2d(), m_chassis.getRotation2d()
                ),

                new PathPoint(current.getTranslation().plus(new Translation2d(2, 0)), new Rotation2d(), current.getRotation())
        );

        AutonCommand command = autonCommandGenerator(trajectory);
        return new SequentialCommandGroup(new ToggleGrabber(m_manipulator), wrapCmd(command));
    }

    public CommandBase makeCmdToGoBackwardsClampAndForwards(Pose2d curr) {
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                violent_constraints,
                new PathPoint(
                        curr.getTranslation(),
                        new Rotation2d(0), m_chassis.getRotation2d()
                ),

                new PathPoint(curr.getTranslation().plus(new Translation2d(-2, 0)), new Rotation2d(0), curr.getRotation()),
                new PathPoint(curr.getTranslation(), new Rotation2d(0), curr.getRotation())
        );

        AutonCommand command = autonCommandGenerator(trajectory);
        return new SequentialCommandGroup(new ToggleIntake(m_intake), wrapCmd(command));
    }

    public CommandBase generateMidPlaceBottomStart() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("place mid bottom", safe_constraints);
        AutonCommand command = autonCommandGeneratorPlacement(trajectory);
        return wrapCmd(command);
    }
}
