package frc.robot.supportingClasses.Auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
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
import frc.robot.commands.Placement.presets.GoToLowScoring;
import frc.robot.commands.Placement.presets.GoToMidScoringCones;
import frc.robot.commands.Placement.presets.GoToPickupWithinBot;
import frc.robot.subsystems.*;

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

    protected final DoNotAskMap<String, Command> eventMap;
    protected final SwerveAutoBuilder autoBuilder;

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

        Shuffleboard.getTab("Comp").add(m_autonChooser);

        m_intake = intake;
        this.rotary = rotary;
        this.extension = extension;
        m_manipulator = manipulator;

        eventMap = new DoNotAskMap<>(); // don't ask why it's called that
        eventMap.put("manipulator", new ToggleManipulator(m_manipulator));
        eventMap.put("zero", new SequentialCommandGroup(new AutoZeroExtensionArm(extension), new AutoZeroRotryArm(rotary)));
        eventMap.put("pickup", new GoToPickupWithinBot(extension));
        eventMap.put("place high", new GoToHighScoring(rotary, extension));
        eventMap.put("grabber", new ToggleManipulator(m_manipulator));
        eventMap.put("place mid", new GoToMidScoringCones(rotary, extension));
        eventMap.put("place low", new GoToLowScoring(rotary, extension));

        autoBuilder = new SwerveAutoBuilder(
                m_chassis::getPose2d,
                m_chassis::resetOdometry,
                m_chassis.getKinematics(),
                new PIDConstants(Constants.kPXController, Constants.kIXController, Constants.kDXController),
                new PIDConstants(Constants.kPThetaController, Constants.kIThetaController, Constants.kDThetaController),
                m_chassis::setModuleStates,
                eventMap,
                true,
                m_chassis
        );

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
        m_autonChooser.addOption("pull out", generatePullOut()); // as the name suggests its the safest option
        
        m_autonChooser.addOption("marker path 2 cones HP", placeConeHighPlaceCubeHigh());
        m_autonChooser.addOption("marker path 2 cones non-hp", loadTrajectory("place two cones non hp"));
    }

    /**
     * what to command is currently selected on shuffleboard.
     * @return the command that is selected on shuffleboard
     */
    public CommandBase pick() {
        return m_autonChooser.getSelected();
    }

    /**
     * A method to help with on the fly trajectory generation. 
     * As of current it will still update position with april tags however this may need to be changed for better effect in the future.
     * 
     * @param current the current position of the bot / start position of the trajectory
     * @param endPoint the end position of the bot / end position of the trajectory
     * @return the Auton command without placement support that will follow the path.
     */
    public CommandBase onTheFlyGenerator(Pose2d current, Pose2d endPoint) {
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(violent_constraints,
                new PathPoint(current.getTranslation(), new Rotation2d(), current.getRotation()),
                new PathPoint(endPoint.getTranslation(), new Rotation2d(), endPoint.getRotation())
        );

        return autoBuilder.followPath(trajectory);
    }

    /**
     * loads a PathPlanner trajectory with optional marker support.
     * If you don't want marker support then this command won't require placement subsystems.
     * Any auton commands that are made using this command will use april tags.
     * 
     * @param nameOfFile the name of the path in PathPlanner
     * @return the generated auton command which requires placement and uses april tags
     */
    public CommandBase loadTrajectory(String nameOfFile) {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(nameOfFile, safe_constraints); //TODO: Make this violent constraints
        return autoBuilder.followPath(trajectory);
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

        return autoBuilder.followPath(trajectory);
    }

    /**
     * on-the-fly generates a path to go to the closest location where you can place a game element from
     * @param current the current robots position or the position of the robot when the path will start
     * @return the generated path as an auton command
     */
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

        return autoBuilder.followPath(trajectory);
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

        return autoBuilder.followPath(trajectory);
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

        return autoBuilder.followPath(trajectory);
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

        CommandBase command = autoBuilder.followPath(trajectory);
        CommandBase command2 = autoBuilder.followPath(trajectory2);
        return new SequentialCommandGroup(
                new FunctionalCommand(() -> m_chassis.setAprilTagUsage(false), () -> {}, (Boolean bool) -> {}, () -> true, m_chassis),
                    new ToggleManipulator(m_manipulator),
                    new GoToHighScoring(rotary, extension),
                    command,
                    new ToggleManipulator(m_manipulator),
                    command2,
                    new AutoZeroExtensionArm(extension),
                    new AutoZeroRotryArm(rotary),
        new FunctionalCommand(() -> m_chassis.setAprilTagUsage(Constants.useAprilTags), () -> {}, (Boolean bool) -> {}, () -> true, m_chassis));
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

        CommandBase command = autoBuilder.followPath(trajectory);
        CommandBase command2 = autoBuilder.followPath(trajectory2);
        return new SequentialCommandGroup(
                new FunctionalCommand(() -> m_chassis.setAprilTagUsage(false), () -> {}, (Boolean bool) -> {}, () -> true, m_chassis),
                    new ToggleManipulator(m_manipulator),
                    new GoToHighScoring(rotary, extension),
                    command,
                    new ToggleManipulator(m_manipulator),
                    command2,
                    new AutoZeroExtensionArm(extension),
                    new AutoZeroRotryArm(rotary),
        new FunctionalCommand(() -> m_chassis.setAprilTagUsage(Constants.useAprilTags), () -> {}, (Boolean bool) -> {}, () -> true, m_chassis));
    }

    /**
     * Place a cone in high at start and then place a cube in high.
     * @return the auton command for the generated trajectory wrapped
     */
    public CommandBase placeConeHighPlaceCubeHigh() {
        PathPlannerTrajectory trajectoryHP = PathPlanner.loadPath("place two cones high hp", new PathConstraints(1.5, 1.5));
        return autoBuilder.followPath(trajectoryHP);
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

        new PathPoint(new Translation2d(1.5, 0), new Rotation2d(0), new Rotation2d(0)));

        return autoBuilder.followPath(trajectory);
    }

}