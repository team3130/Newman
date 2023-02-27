package frc.robot.supportingClasses.CvTele;

import frc.robot.Newman_Constants.Constants.Camera;
import frc.robot.subsystems.Chassis;

import java.util.function.Predicate;

/**
 * The result of hours of coping
 */
public class GameElementManager {
    // cyclical queue for game elements we currently think are in places
    protected final GameElement[] cyclicalQueue;
    protected final int bucketSize;
    protected int lowerBound;
    protected int upperBound;

    // on ground
    protected final GameElement[] elementsOnGround;
    // on grid (3x9) so 27
    protected final GameElement[] elementsOnGrid;

    // check if a game element should be visible from the current camera position
    protected final Predicate<GameElement> withinView;

    // thread
    protected final Thread m_managerThread;

    // functional statemachine
    protected final Runnable[] stateMachine;
    protected int state;

    // constants for the functional statemachine
    protected final static int ADDING_GAME_ELEMENTS = 0;
    protected final static int GENERATING_PATH = 1;
    protected final static int UPDATE_TO_ADD = 2; // default state

    protected final Chassis m_chassis;

    /**
     * Makes a new GameElementManager
     */
    public GameElementManager(Chassis chassis) {
        // cyclical queue stuff
        bucketSize = 16;
        cyclicalQueue = new GameElement[bucketSize];
        lowerBound = 0;
        upperBound = 0;

        // chassis
        m_chassis = chassis;

        // active arrays
        elementsOnGrid = new GameElement[27];

        // withing view stuff
        withinView = (GameElement element) ->
                Math.abs(Math.atan2(
                        element.getY() - m_chassis.getPose2d().getY(),
                        element.getX() - m_chassis.getPose2d().getX()) - m_chassis.getPose2d().getRotation().getRadians())
                        <= Math.toRadians(Camera.kCameraFOV / 2);


        // state machine stuff
        stateMachine = new Runnable[] {this::addGameElementsFromQueue, this::generatePath, this::updateAddQueue};
        state = UPDATE_TO_ADD; // default state is to update stuff from network tables

        // thread that manages everything
        m_managerThread = new Thread(this::manager, "manager");
    }

    public void manager() {
        stateMachine[state].run();
    }

    public void addGameElement(GameElement element) {
        cyclicalQueue[(upperBound++ & 15)] = element;
    }

    public void updateAddQueue() {

    }

    public void generatePath() {

        state = UPDATE_TO_ADD;
    }

    public void addGameElementsFromQueue() {


        state = UPDATE_TO_ADD;
    }

    public GameElement getIdealNextElementToPickup() {

    }

    public int getIdealStoringPosition() {

    }


}
