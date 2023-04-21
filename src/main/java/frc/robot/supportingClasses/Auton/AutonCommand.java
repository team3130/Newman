package frc.robot.supportingClasses.Auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Newman_Constants.Constants;
import frc.robot.commands.DoNothing;
import frc.robot.commands.Manipulator.ToggleManipulator;
import frc.robot.commands.Placement.AutoZeroExtensionArm;
import frc.robot.commands.Placement.AutoZeroRotryArm;
import frc.robot.commands.Placement.presets.*;
import frc.robot.commands.TimedCommand;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ExtensionArm;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.RotaryArm;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Function;

/**
 * Stores the command to run as well as the start and end position of the bot as {@link Pose2d} objects.
 * objects of this class are frequently used for re-setting odometry to a position.
 * <p>
 * Also has capabilities of running along a trajectory.
 */
public class AutonCommand extends CommandBase {

    /**
     * start position
     */
    protected Pose2d startPosition;

    /**
     * end position
     */
    protected Pose2d endPosition;

    /**
     * the trajectory to follow
     */
    protected final PathPlannerTrajectory trajectory;

    /**
     * the holonomic controller command that follows the {@link PathPlannerTrajectory}
     */
    protected final HolonomicControllerCommand cmd;

    /**
     * the chassis subsystem.
     * Should be a singleton.
     * Gets used to run the actual path.
     */
    protected final Chassis m_chassis;

    /**
     * the rotary arm subsystem.
     * Is used for placement markers.
     */
    protected final RotaryArm m_rotaryArm;

    /**
     * the manipulator subsystem.
     * Is used for grabber markers.
     */
    protected final Manipulator m_manipulator;

    /**
     * the extension arm subsystem.
     * Is used for placement markers.
     */
    protected final ExtensionArm m_extensionArm;

    /**
     * An array of all the markers in the path.
     * There are events that are correlated to commands through search or magic scalars.
     */
    protected final ArrayList<EventMarker> markers; // the markers for other events

    /**
     * whether to use optimized stuffs for bin search.
     * Gets set to true when {@link #optimize()} is successful
     */
    protected boolean useOptimized = false;

    /**
     * the magic scalar for the optimization.
     * Gets a value from {@link #optimize()}
     */
    protected double magicScalar;

    /**
     * A pointer to the timer that exists in {@link HolonomicControllerCommand}
     */
    protected final Timer m_timer;

    /**
     * the marker's mapped to the command to run.
     * Gets updated in {@link #mapMarkersToCommands()}
     */
    protected final HashMap<EventMarker, Integer> markerToCommandMap;

    /**
     * The indices of commands that we are running right now.
     */
    protected final ArrayList<Integer> indicesToRun = new ArrayList<>(7); // what indices of commands run right now

    /**
     * A function to get the index using the optimize function given an event marker. Returns the index in {@link #commands}
     */
    protected final Function<EventMarker, Integer> getOptimizedIndex;

    /**
     * The current/most recent event that we are reading
     */
    protected EventMarker current;

    /**
     * Commands that we can potentially run all in an array for quick access either via a map as in:
     * {@link #markerToCommandMap} or with a magic scalar that gets multiplied by time like in {@link #getOptimizedIndex}
     */
    protected CommandBase[] commands;

    /**
     * Whether to use april tags or not.
     * If not then we should reset odometry to the start of the trajectory on initialize.
     * We should also configure Chassis to not update odometry with april tags during the path.
     */
    protected final boolean useAprilTags;

    /**
     * THE CONSTRUCTOR for auton command
     * @param cmd command that will be run during auton
     * @param startPosition the start position of the command
     * @param endPosition the end position of the command
     * @param trajectory the trajectory that we will follow
     * @param rotaryArm the rotary arm subsystem
     * @param extensionArm the extension arm subsystem
     * @param chassis the chassis subsystem
     * @param manipulator the manipulator subsystem
     * @param useAprilTags whether to use april tags or not
     */
    public AutonCommand(HolonomicControllerCommand cmd, Pose2d startPosition, Pose2d endPosition, PathPlannerTrajectory trajectory,
                        RotaryArm rotaryArm, ExtensionArm extensionArm, Chassis chassis, Manipulator manipulator, boolean useAprilTags) {
        this.trajectory = trajectory;
        this.cmd = cmd;
        this.endPosition = endPosition;
        this.startPosition = startPosition;
        m_rotaryArm = rotaryArm;
        m_extensionArm = extensionArm;

        m_chassis = chassis;
        m_manipulator = manipulator;
        m_timer = cmd.getTimer();

        final List<EventMarker> markerList = trajectory.getMarkers();

        markers = new ArrayList<>(markerList.size() + 2);

        markers.add(EventMarker.fromTime(List.of("DoNothing"), 0));
        markers.addAll(markerList);
        markers.add(EventMarker.fromTime(List.of("DoNothing"), trajectory.getTotalTimeSeconds() * 100)); // set really far away so that the bin search thread doesn't hang

        markerToCommandMap = new HashMap<>();

        if (chassis != null) {
            m_requirements.add(chassis);
        }

        if (m_rotaryArm != null && m_extensionArm != null && m_manipulator != null) {
            m_requirements.addAll(List.of(m_rotaryArm, m_extensionArm, m_manipulator));
        }

        CommandBase PLACE_LOW = null, PLACE_MID = null,
                PLACE_HIGH = null, ZERO = null, MANIPULATOR = null,
                PICK_UP_OFF_GROUND = null, PICK_UP_IN_BOT = null,
                DO_NOTHING = new DoNothing(); // sits on index 7

        if (m_rotaryArm != null && m_extensionArm != null && m_manipulator != null) {
            PLACE_LOW = new GoToLowScoring(m_rotaryArm, m_extensionArm); // index: 0

            PLACE_MID = new GoToMidScoringCones(m_rotaryArm, m_extensionArm); // index: 1

            PLACE_HIGH = new GoToHighScoring(m_rotaryArm, m_extensionArm); // index: 2

            MANIPULATOR = new ToggleManipulator(manipulator); // index: 4

            ZERO = new SequentialCommandGroup(new AutoZeroExtensionArm(extensionArm), new AutoZeroRotryArm(rotaryArm)); // index: 3

            PICK_UP_OFF_GROUND = new GoToPickupOffGround(m_rotaryArm, m_extensionArm); // index: 6

            PICK_UP_IN_BOT = new SequentialCommandGroup(new GoToPickupWithinBot(m_extensionArm),
                    new ToggleManipulator(m_manipulator), new TimedCommand(0.1), new AutoZeroExtensionArm(extensionArm), new AutoZeroRotryArm(rotaryArm)); // index: 5

            CommandScheduler.getInstance().registerComposedCommands(PLACE_LOW, PLACE_MID, PLACE_HIGH, MANIPULATOR, ZERO, PICK_UP_IN_BOT, PICK_UP_OFF_GROUND);
        }
        CommandScheduler.getInstance().registerComposedCommands(DO_NOTHING);

        commands = new CommandBase[] {PLACE_LOW, PLACE_MID, PLACE_HIGH, ZERO, MANIPULATOR,
                PICK_UP_IN_BOT, PICK_UP_OFF_GROUND, DO_NOTHING};

        getOptimizedIndex = (EventMarker marker) -> (int) (marker.timeSeconds * magicScalar);

        mapMarkersToCommands();

        this.useAprilTags = useAprilTags;
    }

    /**
     * Map each marker to a specific command.
     */
    protected void mapMarkersToCommands() {
        for (EventMarker marker : markers) {
            for (String name : marker.names) {
                name = name.toLowerCase();
                if (name.contains("place")) {
                    if (name.contains("low")) {
                        markerToCommandMap.put(marker, 0);
                    }
                    if (name.contains("mid")) {
                        markerToCommandMap.put(marker, 1);
                    }
                    if (name.contains("high")) {
                        markerToCommandMap.put(marker, 2);
                    }
                } else if (name.contains("within") || name.contains("bot")) {
                    markerToCommandMap.put(marker, 5);
                } else if (name.contains("pickup") || name.contains("floor")) {
                    markerToCommandMap.put(marker, 6);
                } else if (name.contains("zero")) {
                    markerToCommandMap.put(marker, 3);
                } else if (name.contains("grabber") || name.contains("manipulator")) {
                    markerToCommandMap.put(marker, 4);
                }
                else {
                    markerToCommandMap.put(marker, 7);
                }
            }
        }
    }

    /**
     * Constructs an auton command with certain subsystems that it will need
     * @param cmd the command to run for auton
     * @param trajectory the trajectory to follow
     * @param chassis the chassis subsystem
     * @param rotate the rotary arm subsystem
     * @param extendy the extension arm subsystem
     * @param manipulator the manipulator subsystem
     * @param useAprilTags whether to use april tags or not
     */
    public AutonCommand(HolonomicControllerCommand cmd, PathPlannerTrajectory trajectory, Chassis chassis, RotaryArm rotate, ExtensionArm extendy, Manipulator manipulator, boolean useAprilTags) {
        this(cmd, trajectory.getInitialPose(), trajectory.getEndState().poseMeters, trajectory, rotate,
                extendy, chassis, manipulator, useAprilTags);
    }

    /**
     * Wrapper constructor. Creates an auton command without placement support.
     * @param cmd auton path command
     * @param trajectory auton path
     * @param chassis chassis subsystem
     * @param useAprilTags whether to use april tags or not
     */
    public AutonCommand(HolonomicControllerCommand cmd, PathPlannerTrajectory trajectory, Chassis chassis, boolean useAprilTags) {
        this(cmd, trajectory, chassis, null, null, null, useAprilTags);
    }

    /**
     * Determines the closest marker with the optimize function.
     * Takes the magic scalar and multiplies by the current time to get the index from markers.
     * @return the closest event marker with optimized crap
     */
    protected PathPlannerTrajectory.EventMarker findClosestWithOptimized() {
        if (markers.size() <= 1) {
            return null;
        }

        // the only real line everything else is protection
        int index = (int) (magicScalar * m_timer.get());

        if (index < 0) {
            return markers.get(0);
        }
        if (index >= markers.size()) {
            return markers.get(markers.size() - 1);
        }
        return markers.get(index);
    }

    /**
     * Gets the closes event marker with a binary search.
     * @return the closest event using a binary search
     */
    protected PathPlannerTrajectory.EventMarker findClosestWithBinSearch() {
        if (markers.size() == 0) {
            return null;
        }
        else if (markers.size() == 1) {
            return markers.get(0);
        }

        int low = 0;
        int high = markers.size() - 1;
        // bin search for closest one
        //TODO: DOESN'T WORK
        while (low != high) {
            int midPosition = (low + high) / 2 ;
            if (m_timer.get() >= markers.get(midPosition).timeSeconds && m_timer.get() < markers.get(midPosition + 1).timeSeconds) {
                return markers.get(midPosition);
            }
            if (m_timer.get() < markers.get(midPosition).timeSeconds) {
                //TODO: NEVER RUNS
                high = midPosition;
                continue;
            }
            if (m_timer.get() > markers.get(midPosition).timeSeconds) {
                //TOD: NEVER RUNS
                low = midPosition;
            }
        }
        return markers.get(0);
    }

    /**
     * Needs to be run in its own thread or at least one that wasn't made by the scheduler.
     * Speeds up logic by not needing a binary search.
     *
     * @return if it can be optimized or not.
     */
    public boolean optimize() {
        for (double scalar = 0.1; scalar < 15; scalar += 0.1) {
            if (scalarWorked(scalar)) {
                magicScalar = scalar;
                useOptimized = true;
                return true;
            }
        }
        useOptimized = false;
        return false;
    }

    /**
     * Test method for each scalar to avoid bin search
     * @param scalar the scalar to use
     * @return if the scalar worked
     */
    protected boolean scalarWorked(double scalar) {
        boolean[] test = new boolean[markers.size()];

        for (PathPlannerTrajectory.EventMarker marker : markers) {
            int index = (int) (marker.timeSeconds * scalar);
            if (index < 0) {
                index = 0;
            }
            if (index >= test.length) {
                index = test.length - 1;
            }
            test[index] = true;
        }

        for (boolean bool : test) {
            if (!bool) {
                return false;
            }
        }
        return true;
    }

    /**
     * {@link #cmd}
     * @return the command to run
     */
    public CommandBase getCmd() {
        return cmd;
    }

    /**
     * To initialize portion of the command that runs once when it is scheduled.
     * If there are markers, then it will set the current marker to the first one.
     * If we are suppposed to use april tags then it will make sure that chassis is set to update odometry with april tags.
     * If we are not it will make sure that it is set to update odometry with encoders and it will set odometry to the start of the trajectory.
     */
    @Override
    public void initialize() {
        // ADD SOME CLAUSE FOR IF WE HAVEN'T SEEN A TARGET YET
        if (!useAprilTags) {
            m_chassis.resetOdometry(startPosition);
        }
        m_chassis.setAprilTagUsage(useAprilTags);

        cmd.initialize();
        indicesToRun.clear();

        if (markers.size() >= 1) {
            current = markers.get(0);
        }
    }

    /**
     * The execute portion of the command.
     * This section of the command handles the majority of the logic of cycling through commands based off of markers.
     */
    @Override
    public void execute() {
        // autony execute
        cmd.execute();
        
        // for every command that we currently want to run, run its execute and check if it si finished and handle end
        for (int i = 0; i < indicesToRun.size(); i++) {
            // if there is a command that we are supposed to run right now, then run it until it ends
            commands[indicesToRun.get(i)].execute();
            if (commands[indicesToRun.get(i)].isFinished()) {
                commands[indicesToRun.get(i)].end(false);
                indicesToRun.remove(i--);
            }
        }

        PathPlannerTrajectory.EventMarker closest;
        // event markers
        if (useOptimized) {
            closest = findClosestWithOptimized();
        } else {
            closest = findClosestWithBinSearch();
        }

        // if a new marker has been stumbled across. Should only get ran when we want to initialize markers
        if (closest != current) {
            int toAdd = getIndexFromMap(closest);
            // for debugging purposes this doesn't currently get ran
            if (closest.names.get(0).contains("end")) {
                 commands[toAdd].end(true);
                // remove the marker that is running right now
                for (int i = 0; i < indicesToRun.size(); i++) {
                    if (commands[indicesToRun.get(i)] == commands[toAdd]) {
                        // remove and keep the loop running
                        indicesToRun.remove(i--);
                    }
                }
            }
            // for our purposed read this one
            else {
                if (commands[toAdd] != null) {
                    commands[toAdd].initialize();
                    if (useOptimized) {
                        indicesToRun.add(toAdd);
                    } else {
                        indicesToRun.add(toAdd);
                    }
                }
            }
            current = closest;
        }
    }

    /**
     * Gets the index that represents the command to run in {@link #commands}. uses optimized if available
     * @param marker the key
     * @return the result of the key in the map
     */
    private int getIndexFromMap(EventMarker marker) {
        if (marker == null) {
            return commands.length - 1;
        }
        if (useOptimized) {
            return getOptimizedIndex.apply(marker);
        }
        else {
            return markerToCommandMap.get(marker);
        }

    }

    /**
     * Determines if the command is finished using the commands in the group
     * @return Whether the command is finished based off if every command this is running is done
     */
    @Override
    public boolean isFinished() {
        boolean runningIsDone = true;
        for (int index : indicesToRun) {
            if (!commands[index].isFinished()) {
                runningIsDone = false;
            }
        }
        return cmd.isFinished();
    }

    /**
     * stops the {@link HolonomicControllerCommand} that this is running and stops any other commands in this group
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        cmd.end(interrupted);
        System.out.println("stopping modules");
        m_chassis.stopModules();

        for (int index : indicesToRun) {
            commands[index].end(interrupted);
        }

        m_chassis.setAprilTagUsage(Constants.useAprilTags); // resets april tag usage to it's default state
    }

    /**
     * {@link #trajectory}
     * @return the containing trajectory that this command follows
     */
    public PathPlannerTrajectory getTrajectory() {
        return trajectory;
    }

    /**
     * {@link #startPosition}
     * @return the start position on the field of the trajectory that this command follows
     */
    public Pose2d getStartPosition() {
        return startPosition;
    }

    /**
     * {@link #trajectory}'s initial state's holonomic rotation
     * @return the starting holonomic rotation of the contained trajectory that this command follows
     */
    public Rotation2d getStartRotation() {
        return trajectory.getInitialState().holonomicRotation;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("timer has elapsed", () -> m_timer.hasElapsed(trajectory.getTotalTimeSeconds()), null);
        builder.addDoubleProperty("timer", () -> m_timer.getFPGATimestamp(), null);
        builder.addDoubleProperty("Trajectory length", () -> trajectory.getTotalTimeSeconds(), null);
        builder.addDoubleProperty("Holo time until end", () -> cmd.getTimeUntilEnd(), null);
    }
}
