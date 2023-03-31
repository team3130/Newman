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
import frc.robot.commands.Intake.ToggleIntake;
import frc.robot.commands.Manipulator.ToggleManipulator;
import frc.robot.commands.Placement.AutoZeroExtensionArm;
import frc.robot.commands.Placement.AutoZeroRotryArm;
import frc.robot.commands.Placement.presets.GoToHighScoring;
import frc.robot.commands.Placement.presets.GoToPickupWithinBot;
import frc.robot.commands.TimedCommand;
import frc.robot.subsystems.*;

/**
 * A class to generate our auton paths from PathPlanner
 */
public class AutonManager {
    private final SendableChooser<CommandBase> m_autonChooser; // shuffleboard dropdown menu for selecting the path
    protected Chassis m_chassis; // the chassis object

    protected PathConstraints safe_constraints; // safe speeds for testing
    protected PathConstraints violent_constraints; // wild speeds for if we want to go brrrrrrrrr

    private final DriverStation.Alliance alliance;

    protected Intake m_intake;
    protected RotaryArm rotary;
    protected ExtensionArm extension;
    protected Manipulator m_manipulator;

    /**
     * Makes an object to make and manage auton paths.
     * Also calls {@link #populateChooser()}
     * @param chassis needs chassis so that commands made in here can use it
     */
    public AutonManager(Chassis chassis, Intake intake, RotaryArm rotary, ExtensionArm extension, Manipulator manipulator) {
        this.m_autonChooser = new SendableChooser<>();
        this.m_chassis = chassis;

        safe_constraints = new PathConstraints(2, 2);
        violent_constraints = new PathConstraints(Constants.kPhysicalMaxSpeedMetersPerSecond, 3);

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
        // m_autonChooser.addOption("3 Meter", generate3MeterDrive());
        // m_autonChooser.addOption("Question mark", generateExamplePathFromFile());
        // m_autonChooser.addOption("player side", generatepWeekZeroPath());
        // m_autonChooser.addOption("far side", generatepWeekZeroPath2());
        // m_autonChooser.addOption("feelin spicy", generateExamplePathFromPoses());
        // m_autonChooser.addOption("circuit", complexPathTest());
        // m_autonChooser.addOption("AprilTagTesting",aprilTagTesting());
        m_autonChooser.addOption("move out of start intake pushy", makeCmdToIntakeAndGoForward());
//        m_autonChooser.addOption("move out and clamp", generateMoveOutAndClamp());
        // m_autonChooser.addOption("Two meter forward", generateExamplePathFromPoses()); // two meter forward (stable)
        m_autonChooser.addOption("Intake spit", actuateIntake());
//        m_autonChooser.addOption("place in auton", placeInAuton());
        //m_autonChooser.addOption("place in auton top", placeInAutonTop());
//        m_autonChooser.addOption("place in auton", placeInAutonHigh());
        m_autonChooser.addOption("place in auton move out", placeInAutonCone());
        m_autonChooser.addOption("place in auton don't move", placeInAuton());
        m_autonChooser.addOption("pull out", generatePullOut());
        // m_autonChooser.addOption("top dumb", generateTopDumb());
        // m_autonChooser.addOption("bottom dumb", generateBottomDumb());
        // m_autonChooser.addOption("mid placement start top", generateMidPlaceTopStart());

        //m_autonChooser.addOption("marker path <- not for comp", generateMarkerPath());
        m_autonChooser.addOption("marker path cones", placeConeHighPlaceCubeHigh());
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

        // trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, alliance);

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
    public CommandBase generateExamplePathFromPoses() {
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

    public Command aprilTagTesting() {
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
        final double rotation = (DriverStation.getAlliance() == DriverStation.Alliance.Blue) ? Math.PI : 0;
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

    /**
     * @return a command to move out and clamp the manipulator closed
     */
    private CommandBase generateMoveOutAndClamp() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("clamp and move out", safe_constraints);
        CommandBase command = wrapCmd(autonCommandGenerator(trajectory));
        return new SequentialCommandGroup(new AutoZeroExtensionArm(extension), new ToggleManipulator(m_manipulator), command);
    }

    /**
     * @return A command that starts at 0, 0 and actuates intake
     */
    public CommandBase actuateIntake() {
        return new ToggleIntake(m_intake);
    }

    public CommandBase generateTopDumb() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("dumb leave top", safe_constraints);
        CommandBase command = wrapCmd(autonCommandGenerator(trajectory));
        return new SequentialCommandGroup(new ToggleManipulator(m_manipulator), command, new GoToHighScoring(rotary, extension));
    }

    public CommandBase generateBottomDumb() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("dumb leave bottom", safe_constraints);
        CommandBase command = wrapCmd(autonCommandGenerator(trajectory));
        return new SequentialCommandGroup(new ToggleManipulator(m_manipulator), command, new GoToHighScoring(rotary, extension));
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

        AutonCommand command = autonCommandGenerator(trajectory);
        return new SequentialCommandGroup(new ToggleIntake(m_intake), wrapCmd(command));
    }

    public CommandBase makeCmdToGoBackwardsClampAndForwards() {
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                violent_constraints,
                new PathPoint(
                        new Translation2d(0, 0),
                        new Rotation2d(0), new Rotation2d()
                ),

                new PathPoint(new Translation2d(-2, 0), new Rotation2d(0), new Rotation2d(0)),
                new PathPoint(new Translation2d(0, 0), new Rotation2d(0), new Rotation2d(0))
        );

        AutonCommand command = autonCommandGenerator(trajectory);
        return new SequentialCommandGroup(new ToggleManipulator(m_manipulator), wrapCmd(command));
    }

    /**
     * Requires odometry from april tags to be off in auton and for the traajectory to not be transformed by alliance
     * @return a command to place in auton assuming you are starting at the top of the field
     */
    public CommandBase placeInAutonTop() {
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

                new PathPoint(new Translation2d(-1.75, 0), new Rotation2d(0), new Rotation2d(0))
        );

        AutonCommand command = autonCommandGenerator(trajectory);
        AutonCommand command2 = autonCommandGenerator(trajectory2);
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
    public CommandBase placeInAuton() {
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

                new PathPoint(new Translation2d(0, 0), new Rotation2d(0), new Rotation2d(0))
        );

        AutonCommand command = autonCommandGenerator(trajectory);
        AutonCommand command2 = autonCommandGenerator(trajectory2);
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
     * @return A command to place high assuming that we start in the middle of the field
     */
    public CommandBase generateMidPlaceBottomStart() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("place mid bottom", safe_constraints);
        AutonCommand command = autonCommandGeneratorPlacement(trajectory);
        return wrapCmd(command);
    }

    /**
     * @return place in auton assuming that we start on the non-human player side
     */
    public CommandBase placeInAutonLower() {
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

                new PathPoint(new Translation2d(-4, 0), new Rotation2d(0), new Rotation2d(0))
        );

        CommandBase command = autonCommandGenerator(trajectory);
        AutonCommand command2 = autonCommandGenerator(trajectory2);

        if (Constants.debugMode) {
            command = wrapCmd((AutonCommand) command);
        }

        return
            new SequentialCommandGroup(new ToggleManipulator(m_manipulator), new GoToHighScoring(rotary, extension),
            command, new ToggleManipulator(m_manipulator),
            new ParallelCommandGroup(
                    command2,
                    new SequentialCommandGroup(
                            new AutoZeroExtensionArm(extension),
                            new AutoZeroRotryArm(rotary))));
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

                new PathPoint(new Translation2d(-4, 0), new Rotation2d(0), new Rotation2d(0))
        );

        AutonCommand command = autonCommandGenerator(trajectory);
        AutonCommand command2 = autonCommandGenerator(trajectory2);

        CommandBase command1 = wrapCmd(command);

        return
            new SequentialCommandGroup(
                new AutoZeroRotryArm(rotary),
                new AutoZeroExtensionArm(extension),
                new GoToPickupWithinBot(extension),
                new ToggleManipulator(m_manipulator),
                new TimedCommand(0.2),
                new AutoZeroExtensionArm(extension),
                new GoToHighScoring(rotary, extension),
                command1,
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
     * Place a cone in high at start and then place a cube in high
     * @return the auton command
     */
    public CommandBase placeConeHighPlaceCubeHigh() {
        PathPlannerTrajectory trajectoryHP = PathPlanner.loadPath("place cone high place cube high hp", safe_constraints);
        AutonCommand commandHP = autonCommandGeneratorPlacement(trajectoryHP);

/*        PathPlannerTrajectory trajectorynonHP = PathPlanner.loadPath("place cone high place cube high non hp", safe_constraints);
        AutonCommand commandnonHP = autonCommandGeneratorPlacement(trajectorynonHP);*/

        return wrapCmd(commandHP);
    }

    /**
     * place in auton high
     * @return a PoseCommand for placing in auton
     */
    public CommandBase placeInAutonHigh() {
        PathPlannerTrajectory trajectoryHP = PathPlanner.loadPath("place cone start hp", violent_constraints);
        PathPlannerTrajectory trajectorynonHP = PathPlanner.loadPath("place cone start non hp", violent_constraints);

        AutonCommand commandHP = autonCommandGeneratorPlacement(trajectoryHP);
        AutonCommand commandNonHp = autonCommandGeneratorPlacement(trajectorynonHP);

        return wrapCmd(commandHP);
    }
}

// public void generatePullOut() {
//     PathPlannerTrajectory trajectory = PathPlanner.generatePath(safe_constraints, 
//     new PathPoint()
//     );
// }
