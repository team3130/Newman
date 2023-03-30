package frc.robot.supportingClasses.Auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DoNothing;
import frc.robot.commands.Hopper.SpinHopper;
import frc.robot.commands.Manipulator.ToggleManipulator;
import frc.robot.commands.Placement.AutoZeroExtensionArm;
import frc.robot.commands.Placement.AutoZeroRotryArm;
import frc.robot.commands.Placement.presets.GoToHighScoring;
import frc.robot.commands.Placement.presets.GoToLowScoring;
import frc.robot.commands.Placement.presets.GoToMidScoringCube;
import frc.robot.commands.Placement.presets.*;
import frc.robot.commands.TimedCommand;
import frc.robot.commands.Placement.presets.*;
import frc.robot.commands.TimedCommand;
import frc.robot.subsystems.*;

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
public class AutonCommand extends CommandBase{
    protected Pose2d startPosition; // start position
    protected Pose2d endPosition; // end position
    protected final PathPlannerTrajectory trajectory; // the trajectory to follow
    protected final HolonomicControllerCommand cmd;
    protected final Chassis m_chassis; // the chassis subsystem
    protected final Hopper m_hopper; // the hopper subsystem
    protected final RotaryArm m_rotaryArm; // the rotary arm subsystem
    protected final Manipulator m_manipulator; // the manipulator subsystem
    protected final ExtensionArm m_extensionArm; // the extension arm subsystem

    protected final ArrayList<EventMarker> markers; // the markers for other events

    protected boolean useOptimized = false; // whether to use optimized stuffs for bin search
    protected double magicScalar; // the magic scalar for the optimization

    protected final Timer m_timer; // timer from the holonomic drive command

    protected final HashMap<EventMarker, Integer> markerToCommandMap; // the marker's mapped to the command to run
    protected final ArrayList<Integer> indicesToRun = new ArrayList<>(8); // what indices of commands run right now

    protected final Function<EventMarker, Integer> getOptimizedIndex; // gets the index using the optimize function

    protected EventMarker current; // current event we are reading

    protected CommandBase[] commands; // commands necessary

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
     * @param hopper the hopper subsystem
     */
    public AutonCommand(HolonomicControllerCommand cmd, Pose2d startPosition, Pose2d endPosition, PathPlannerTrajectory trajectory,
                        RotaryArm rotaryArm, ExtensionArm extensionArm, Chassis chassis, Manipulator manipulator,
                        Hopper hopper) {
        this.trajectory = trajectory;
        this.cmd = cmd;
        this.endPosition = endPosition;
        this.startPosition = startPosition;
        m_rotaryArm = rotaryArm;
        m_extensionArm = extensionArm;
        markers = (ArrayList<EventMarker>) trajectory.getMarkers();
        m_chassis = chassis;
        m_manipulator = manipulator;
        m_hopper = hopper;
        m_timer = cmd.getTimer();

        markerToCommandMap = new HashMap<>();

        if (chassis != null) {
            m_requirements.add(chassis);
        }

        if (hopper != null) {
            m_requirements.add(m_hopper);
        }

        if (m_rotaryArm != null && m_extensionArm != null && m_manipulator != null) {
            m_requirements.addAll(List.of(m_rotaryArm, m_extensionArm, m_manipulator));
        }

        CommandBase HOPPER = null, PLACE_LOW = null, PLACE_MID = null, PLACE_HIGH = null, ZERO = null, PICK_UP_OFF_GROUND = null, PICK_UP_IN_BOT = null, DO_NOTHING = null;
        CommandBase HOPPER = null, PLACE_LOW = null, PLACE_MID = null,
                PLACE_HIGH = null, ZERO = null, MANIPULATOR = null,
                PICK_UP_OFF_GROUND = null, PICK_UP_IN_BOT = null,
                DO_NOTHING = null;

        if (m_hopper != null) {
            HOPPER = new SpinHopper(m_hopper);
            CommandScheduler.getInstance().registerComposedCommands(HOPPER);
        }

        if (m_rotaryArm != null && m_extensionArm != null && m_manipulator != null) {
            PLACE_LOW = new GoToLowScoring(m_rotaryArm, m_extensionArm);

            PLACE_MID = new SequentialCommandGroup(
                    new ToggleManipulator(m_manipulator), new GoToMidScoringCube(m_rotaryArm, m_extensionArm),
                    new ToggleManipulator(m_manipulator), new AutoZeroRotryArm(m_rotaryArm), new AutoZeroExtensionArm(m_extensionArm)
            );
            PLACE_MID = new GoToMidScoring(m_rotaryArm, m_extensionArm);

            PLACE_HIGH = new GoToHighScoring(m_rotaryArm, m_extensionArm);

            MANIPULATOR = new ToggleManipulator(manipulator);

            ZERO = new SequentialCommandGroup(new AutoZeroExtensionArm(extensionArm), new AutoZeroRotryArm(rotaryArm));

            PICK_UP_OFF_GROUND = new GoToPickupOffGround(m_rotaryArm, m_extensionArm);

            PICK_UP_IN_BOT = new SequentialCommandGroup(new GoToPickupWithinBot(m_extensionArm),
                    new ToggleManipulator(m_manipulator), new TimedCommand(0.1), new AutoZeroExtensionArm(extensionArm), new AutoZeroRotryArm(rotaryArm));

            CommandScheduler.getInstance().registerComposedCommands(PLACE_LOW, PLACE_MID, PLACE_HIGH, MANIPULATOR, ZERO, PICK_UP_IN_BOT, PICK_UP_OFF_GROUND);
        }

        DO_NOTHING = new DoNothing();
        CommandScheduler.getInstance().registerComposedCommands(DO_NOTHING);

        commands = new CommandBase[] {HOPPER, PLACE_LOW, PLACE_MID, PLACE_HIGH, ZERO, MANIPULATOR,
                PICK_UP_OFF_GROUND, PICK_UP_IN_BOT, DO_NOTHING};

        getOptimizedIndex = (EventMarker marker) -> (int) (marker.timeSeconds * magicScalar);

        mapMarkersToCommands();
    }

    /**
     * Map each marker to a specific command
     */
    protected void mapMarkersToCommands() {
        for (EventMarker marker : markers) {
            String name = marker.names.get(0).toLowerCase();
            if (name.contains("intake")) {
                markerToCommandMap.put(marker, 0);
            } else if (name.contains("place")) {
                if (name.contains("low")) {
                    markerToCommandMap.put(marker, 1);
                }
                if (name.contains("mid")) {
                    markerToCommandMap.put(marker, 2);
                }
                if (name.contains("high")) {
                    markerToCommandMap.put(marker, 3);
                }
            }
            else if (name.contains("within") || name.contains("bot")) {
                markerToCommandMap.put(marker, 6);
            }
            else if (name.contains("pickup") || name.contains("floor")) {
                markerToCommandMap.put(marker, 7);
            }
            else if (name.contains("zero")) {
                markerToCommandMap.put(marker, 4);
            }
            else if (name.contains("grabber") || name.contains("manipulator")) {
                markerToCommandMap.put(marker, 5);
            }
            else {
                markerToCommandMap.put(marker, 8);
            }
        }
    }

    /**
     * The second to last constructor for auton command
     * @param cmd the command to follow in auton
     * @param trajectory the trajectory that the command is following
     * @param rotate the rotary arm subsystem
     * @param extendy the extension arm subsystem
     * @param manipulator the manipulator subsystem
     * @param chassis the chassis subsystem
     */
    public AutonCommand(HolonomicControllerCommand cmd, PathPlannerTrajectory trajectory, RotaryArm rotate, ExtensionArm extendy, Manipulator manipulator, Hopper hopper, Chassis chassis) {
        this(cmd, trajectory.getInitialPose(), trajectory.getEndState().poseMeters, trajectory, rotate, extendy, chassis, manipulator, hopper);
    }

    /**
     * Constructs an auton command with certain subsystems that it will need
     * @param cmd the command to run for auton
     * @param trajectory the trajectory to follow
     * @param rotate the rotary arm subsystem
     * @param extendy the extension arm subsystem
     */
    public AutonCommand(HolonomicControllerCommand cmd, PathPlannerTrajectory trajectory, Chassis chassis, RotaryArm rotate, ExtensionArm extendy, Manipulator manipulator) {
        this(cmd, trajectory.getInitialPose(), trajectory.getEndState().poseMeters, trajectory, rotate,
                extendy, chassis, manipulator, null);
    }

    /**
     * Makes a new auton command that stores the start and end positions of the bot as well as the trajectory
     * @param cmd the command to run
     * @param trajectory the trajectory to follow
     */
    public AutonCommand(HolonomicControllerCommand cmd, PathPlannerTrajectory trajectory) {
        this(cmd, trajectory, null);
    }

    /**
     * Wrapper constructor
     * @param cmd auton path command
     * @param trajectory auton path
     * @param chassis chassis subsystem
     */
    public AutonCommand(HolonomicControllerCommand cmd, PathPlannerTrajectory trajectory, Chassis chassis) {
        this(cmd, trajectory, null, null, null, null, chassis);
    }

    /**
     * @return the closest event marker with optimized crap
     */
    protected PathPlannerTrajectory.EventMarker findClosestWithOptimized() {
        if (markers.size() <= 1) {
            return null;
        }
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
     * Gets the closes event marker with a binary search
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
        while (low != high) {
            int midPosition = (low + high) / 2 ;
            if (m_timer.get() >= markers.get(midPosition).timeSeconds || m_timer.get() < markers.get(midPosition + 1).timeSeconds) {
                return markers.get(midPosition);
            }
            if (m_timer.get() < markers.get(midPosition).timeSeconds) {
                high = midPosition;
                continue;
            }
            if (m_timer.get() > markers.get(midPosition).timeSeconds) {
                low = midPosition;
            }
        }
        return null;
    }

    /**
     * Needs to be run in its own thread or at least one that wasn't made by the scheduler.
     * Speeds up logic by not needing a binary search
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
     * @return the command to run
     */
    public CommandBase getCmd() {
        return cmd;
    }

    /**
     * To initialize portion of the command that runs once when it is scheduled
     */
    @Override
    public void initialize() {
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
            System.out.println("index: " + i);
            // if there is a command that we are supposed to run right now, then run it until it ends
            commands[indicesToRun.get(i)].execute();
            if (commands[indicesToRun.get(i)].isFinished()) {
                commands[indicesToRun.get(i)].end(false);
                indicesToRun.remove(i--);
            }
        }

        System.out.println("--------------");

        PathPlannerTrajectory.EventMarker closest;
        // event markers
        if (useOptimized) {
            closest = findClosestWithOptimized();
        } else {
            closest = findClosestWithBinSearch();
        }

        // if a new marker has been stumbled across. Should only get ran when we wan to intialize markers
        if (closest != current) {
            int toAdd = getIndexFromMap(closest);
            System.out.println("current index: " + closest);
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
                commands[toAdd].initialize();
                if (useOptimized) {
                    indicesToRun.add(toAdd);
                }
                else {
                    indicesToRun.add(toAdd);
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

    @Override
    public boolean isFinished() {
        boolean runningIsDone = true;
        for (int index : indicesToRun) {
            if (!commands[index].isFinished()) {
                runningIsDone = false;
            }
        }
        return cmd.isFinished() && runningIsDone;
    }

    @Override
    public void end(boolean interrupted) {
        cmd.end(interrupted);

        for (int index : indicesToRun) {
            commands[index].end(interrupted);
        }
    }

    public PathPlannerTrajectory getTrajectory() {
        return trajectory;
    }

    public Pose2d getStartPosition() {
        return trajectory.getInitialPose();
    }

    public Rotation2d getStartRotation() {
        return trajectory.getInitialState().holonomicRotation;
    }
}
