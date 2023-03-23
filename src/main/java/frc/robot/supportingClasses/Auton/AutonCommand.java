package frc.robot.supportingClasses.Auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DoNothing;
import frc.robot.commands.Hopper.SpinHopper;
import frc.robot.commands.Manipulator.ToggleGrabber;
import frc.robot.commands.Placement.*;
import frc.robot.subsystems.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Function;

/**
 * Stores the command to run as well as the start and end position of the bot as {@link Pose2d} objects.
 * objects of this class are frequently used for re-setting odometry to a position.
 * <p>
 * Also has capabilities of running along a trajectory.
 */
public class AutonCommand extends CommandBase {
    protected Pose2d startPosition; // start position
    protected Pose2d endPosition; // end position
    protected final HolonomicControllerCommand cmd; // command to run
    protected final PathPlannerTrajectory trajectory; // the trajectory to follow

    protected final Chassis m_chassis; // the chassis subsystem
    protected final Hopper m_hopper; // the hopper subsystem
    protected final PlacementRotaryArm m_rotaryArm; // the rotary arm subsystem
    protected final Manipulator m_manipulator; // the manipulator subsystem
    protected final PlacementExtensionArm m_extensionArm; // the extension arm subsystem

    protected final ArrayList<EventMarker> markers; // the markers for other events

    protected boolean useOptimized = false; // whether to use optimized stuffs for bin search
    protected double magicScalar; // the magic scalar for the optimization

    protected final Timer m_timer; // timer from the holonomic drive command

    protected final HashMap<EventMarker, Integer> markerToCommandMap; // the marker's mapped to the command to run
    protected final ArrayList<Integer> indicesToRun = new ArrayList<>(5); // what indices of commands run right now

    protected final Function<EventMarker, Integer> getOptimizedIndex;

    protected EventMarker current; // current

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
                        PlacementRotaryArm rotaryArm, PlacementExtensionArm extensionArm, Chassis chassis, Manipulator manipulator,
                        Hopper hopper) {
        this.cmd = cmd;
        this.trajectory = trajectory;
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

        CommandBase HOPPER = null, PLACE_LOW = null, PLACE_MID = null, PLACE_HIGH = null, DO_NOTHING = null;

        if (hopper != null) {
            HOPPER = new SpinHopper(m_hopper);
        }

        if (m_rotaryArm != null && m_extensionArm != null && m_manipulator != null) {
            PLACE_LOW = new SequentialCommandGroup(
                    new ToggleGrabber(m_manipulator), new LowRotary(m_rotaryArm, m_extensionArm),
                    new ToggleGrabber(m_manipulator), new AutoZeroRotryArm(m_rotaryArm), new ZeroExtension(m_extensionArm)
            );

            PLACE_MID = new SequentialCommandGroup(
                    new ToggleGrabber(m_manipulator), new MidRotary(m_rotaryArm, m_extensionArm),
                    new ToggleGrabber(m_manipulator), new AutoZeroRotryArm(m_rotaryArm), new ZeroExtension(m_extensionArm)
            );

            PLACE_HIGH = new SequentialCommandGroup(
                    new ToggleGrabber(m_manipulator), new HighRotary(m_rotaryArm, m_extensionArm),
                    new ToggleGrabber(m_manipulator), new AutoZeroRotryArm(m_rotaryArm), new ZeroExtension(m_extensionArm)
            );

            DO_NOTHING = new DoNothing();
        }

        commands = new CommandBase[] {HOPPER, PLACE_LOW, PLACE_MID, PLACE_HIGH, DO_NOTHING};

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
            else {
                markerToCommandMap.put(marker, 4);
            }
        }
    }

    /**
     * Uses optimized map if possible to get the command from the marker
     * @param marker a marker from the available markers
     * @return the command from whichever map we are supposed to use rn
     */
    protected CommandBase getCommandFromMap(EventMarker marker) {
        if (useOptimized) {
            return commands[getOptimizedIndex.apply(marker)];
        }
        else {
            return commands[markerToCommandMap.get(marker)];
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
    public AutonCommand(HolonomicControllerCommand cmd, PathPlannerTrajectory trajectory, PlacementRotaryArm rotate, PlacementExtensionArm extendy, Manipulator manipulator, Hopper hopper, Chassis chassis) {
        this(cmd, trajectory.getInitialPose(), trajectory.getEndState().poseMeters, trajectory, rotate, extendy, chassis, manipulator, hopper);
    }

    /**
     * Constructs an auton command with certain subsystems that it will need
     * @param cmd the command to run for auton
     * @param trajectory the trajectory to follow
     * @param rotate the rotary arm subsystem
     * @param extendy the extension arm subsystem
     */
    public AutonCommand(HolonomicControllerCommand cmd, PathPlannerTrajectory trajectory, PlacementRotaryArm rotate, PlacementExtensionArm extendy, Manipulator manipulator) {
        this(cmd, trajectory.getInitialPose(), trajectory.getEndState().poseMeters, trajectory, rotate,
                extendy, null, manipulator, null);
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
        int low = 0;
        int high = markers.size();
        // bin search for closest one
        while (low != high) {
            int midPosition = (low + high) / 2;
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
     * @return the final position in the trajectory
     */
    public Pose2d getEndPosition() {
        return endPosition;
    }

    /**
     * Setter for the end position
     * @param endPosition the last position in the trajectory
     */
    public void setEndPosition(Pose2d endPosition) {
        this.endPosition = endPosition;
    }

    /**
     * sets the start position
     * @param newPosition position to set the start position to
     */
    public void setStartPosition(Pose2d newPosition) {
        startPosition = newPosition;
    }

    /**
     * sets the start position, sets the rotation to 0
     * @param x position (mathematical coordinates)
     * @param y position (mathematical coordinates)
     */
    public void setPosition(double x, double y) {
        setPosition(x, y, 0);
    }

    /**
     * sets the start position
     * @param x position (mathematical coordinates)
     * @param y position (mathematical coordinates)
     * @param rad rotation in radians (mathematical coordinates)
     */
    public void setPosition(double x, double y, double rad) {
        setPosition(x, y, new Rotation2d(rad));
    }

    /**
     * A setter for the start position and assigns it to the {@link Pose2d}
     * @param x position (mathematical coordinates)
     * @param y position (mathematical coordinates)
     * @param rotation rotation (mathematical coordinates)
     */
    public void setPosition(double x, double y, Rotation2d rotation) {
        startPosition = new Pose2d(x, y, rotation);
    }

    /**
     * @return the start position of the auton trajectory
     */
    public Pose2d getStartPosition() {
        return startPosition;
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
    }

    @Override
    public void execute() {
        // autony execute
        cmd.execute();

        for (int i = 0; i < indicesToRun.size(); i++) {
            // if there is a command that we are supposed to run right now, then run it until it ends
            commands[indicesToRun.get(i)].execute();
            if (commands[indicesToRun.get(i)].isFinished()) {
                commands[indicesToRun.get(i)].end(false);
                indicesToRun.remove(indicesToRun.get(i));
            }
        }

        PathPlannerTrajectory.EventMarker closest;
        // event markers
        if (useOptimized) {
            closest = findClosestWithOptimized();
        } else {
            closest = findClosestWithBinSearch();
        }

        if (closest != current) {
            CommandBase toAdd = getCommandFromMap(closest);
            if (closest.names.get(0).contains("end")) {
                toAdd.end(true);
                // remove marker running right now
                for (int i = 0; i < indicesToRun.size(); i++) {
                    if (commands[indicesToRun.get(i)] == toAdd) {
                        indicesToRun.remove(i--);
                    }
                }
            }
            else {
                toAdd.initialize();
            }
            current = closest;
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
}
