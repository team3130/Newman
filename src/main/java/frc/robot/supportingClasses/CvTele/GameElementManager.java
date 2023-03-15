package frc.robot.supportingClasses.CvTele;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Newman_Constants.Constants;
import frc.robot.Newman_Constants.Constants.Camera;
import frc.robot.subsystems.Chassis;
import frc.robot.supportingClasses.CvTele.GameElement.GameElementType;

import java.util.ArrayList;
import java.util.function.Predicate;

/**
 * The result of hours of coping
 */
public class GameElementManager {
    // cyclical queue for game elements that we will add to the heap
    protected final GameElement[] cyclicalQueue;
    protected final int bucketSize; // the size of the queue. This should be
    protected final int clockTick;
    protected int lowerBound; // the lower bound of the moving window (basically head)
    protected int upperBound; // the upper bound of the moving window (basically capacity)

    // funny cached values basically maps
    /*
        Derivation of logic:

        // first index
        if (position0 % 3 == 0) {
            position1 = position0 + 1;
            position2 = position0 + 2;
        }
        // middle index
        else if (position0 % 3 == 1) {
            position1 = position0 - 1;
            position2 = position0 + 1;
        }
        // last index
        else {
            position1 = position0 - 2;
            position2 = position0 - 1;
        }
     */

    // 1, -1, -2
    protected final int[] positionToRowCheckerPosition1 = new int[] {
        1, -1, -2, 1, -1, -2, 1, -1, -2,
        1, -1, -2, 1, -1, -2, 1, -1, -2,
        1, -1, -2, 1, -1, -2, 1, -1, -2
    }; // for check row, position0 is arg. technically could be 3 items but this way we can cut out the mod
    // 2, 1, -1
    protected final int[] positionToRowCheckerPosition2 = new int[] {
        2, 1, -1, 2, 1, -1, 2, 1, -1,
        2, 1, -1, 2, 1, -1, 2, 1, -1,
        2, 1, -1, 2, 1, -1, 2, 1, -1
    }; // for check row, position0 is arg. technically could be 3 items but this way we can cut out the mod

    /*
    5
    3
    2
     */
    protected final int[] positionToRowBonus = new int[] {
            5, 5, 5, 5, 5, 5, 5, 5, 5,
            3, 3, 3, 3, 3, 3, 3, 3, 3,
            2, 2, 2, 2, 2, 2, 2, 2, 2
    }; // for row bonus. top row is 5 points, middle is 3, and bottom is 2. could be 3 items but this is less math


    /*
        Top corners and sides are cones, middle two are cubes, bottom is any
        One grid would look like:
        Cone, Cube, Cone
        Cone, Cube, Cone
        Any , Any, Any
     */
    protected final GameElementType[] positionToGEType = new GameElementType[] {
        GameElementType.CONE, GameElementType.CUBE, GameElementType.CONE, GameElementType.CONE, GameElementType.CUBE, GameElementType.CONE, GameElementType.CONE, GameElementType.CUBE, GameElementType.CONE,
        GameElementType.CONE, GameElementType.CUBE, GameElementType.CONE, GameElementType.CONE, GameElementType.CUBE, GameElementType.CONE, GameElementType.CONE, GameElementType.CUBE, GameElementType.CONE,
        GameElementType.EITHER, GameElementType.EITHER, GameElementType.EITHER, GameElementType.EITHER, GameElementType.EITHER, GameElementType.EITHER, GameElementType.EITHER, GameElementType.EITHER, GameElementType.EITHER,
    };

    // states of the game
    protected boolean haveRankingPoint; // do we have the scoring ranking point

    // game elements on the ground
    protected final GameElement[] elementsOnGround;
    protected final short groundBucketSize; // number of game elements that will realistically be on the ground
    // on grid (3x9) so 27
    protected final GameElement[] elementsOnGrid;
    protected final short gridBucketSize; // number of game elements that can be in the grid

    // check if a game element should be visible from the current camera position.
    //  predicate is a functional interface for checks
    protected final Predicate<Pose2d> withinView;

    // thread that manages execution in the class. basically blocking/queue.
    protected final Thread m_managerThread;

    // functional statemachine
    protected final Runnable[] stateMachine;
    protected int state; // current state of the thread in the statemachine

    // constants for the functional statemachine
    protected final static int ADDING_GAME_ELEMENTS = 0; // adding game elements from toAdd
    protected final static int GENERATING_PATH = 1; // generating an auton path
    protected final static int UPDATE_TO_ADD = 2; // default state

    protected final Chassis m_chassis; // chassis subsystem

    /**
     * Makes a new GameElementManager
     */
    public GameElementManager(Chassis chassis) {
        // cyclical queue stuff
        bucketSize = 128;
        clockTick = bucketSize - 1; // should be the closest value in cache
        cyclicalQueue = new GameElement[bucketSize];
        // initialize both to 0 as there are no elements in the array
        lowerBound = 0;
        upperBound = 0;

        // chassis
        m_chassis = chassis;

        // active arrays
        gridBucketSize = 27;
        groundBucketSize = (short) Math.pow(2, 4); // 2 for base 2, second number is basically how many bits for the clock
        elementsOnGround = new GameElement[groundBucketSize];
        elementsOnGrid = new GameElement[gridBucketSize];

        // withing view stuff
        withinView = (Pose2d otherPose) ->
                Math.abs(Math.atan2(
                        otherPose.getY() - m_chassis.getPose2d().getY(),
                        otherPose.getX() - m_chassis.getPose2d().getX()) - m_chassis.getPose2d().getRotation().getRadians())
                        <= Math.toRadians(Camera.kCameraFOV / 2);


        // state machine stuff
        stateMachine = new Runnable[] {this::addGameElementsFromQueue, this::generatePath, this::updateAddQueue};
        state = UPDATE_TO_ADD; // default state is to update stuff from network tables

        // thread that manages everything
        m_managerThread = new Thread(this::manager, "manager");

        // logic to determine if you can use & for the clock
        if (Long.bitCount(bucketSize) == 1) { // checks number of "1" bits in two's compliment of provided int
            DriverStation.reportError("bucket size is not clock-able", false);
        }

    }

    /**
     * Whether the current position is occupied with another game element or not
     * @param position position on the grid (1->27)
     * @return if the passed in position on the grid is occupied or not
     */
    protected final boolean isOccupied(final int position) {
        return elementsOnGrid[position] == null;
    }

    /**
     * If the row of the passed in position is completed
     * @param position0 position on the grid (1->27)
     * @return if the row is completed on the position on the grid
     */
    protected boolean rowIsCompleted(final int position0) {
        final int position1 = positionToRowCheckerPosition1[position0];
        final int position2 = positionToRowCheckerPosition2[position0];
        return isOccupied(position0) && isOccupied(position1) && isOccupied(position2);
    }

    protected void manager() {
        stateMachine[state].run();
    }

    public void addGameElement(GameElement element) {
        cyclicalQueue[(upperBound++ & 15)] = element;
    }

    public void updateAddQueue() {

    }

    public void generatePath() {
        final int scoringPosition = this.getIdealScoringPosition();
        final GameElement toGet = getIdealNextElementToPickup(scoringPosition, m_chassis.getPose2d());

        state = UPDATE_TO_ADD;
    }

    /**
     * add the game elements to the registry from the queue
     */
    public void addGameElementsFromQueue() {


        state = UPDATE_TO_ADD;
    }

    /**
     * @param position position you are comparing to
     * @param gameElements the game elements you are looking for the closest of
     * @return the closest game element to the passed in position
     */
    protected GameElement getClosestElementTo(Pose2d position, ArrayList<GameElement> gameElements) {
        // search for closest element
        double closestDistance = Integer.MAX_VALUE;
        int closestGameElementInList = -1;
        // selection search of gameElements
        for (int i = 0; i < gameElements.size(); i++) {
            // distance formula distance to a point
            double distanceTo = gameElements.get(i).getDistanceToPosition(position);

            /// if less, then update
            if (distanceTo < closestDistance) {
                closestDistance = distanceTo;
                closestGameElementInList = i;
            }
        }
        return gameElements.get(closestGameElementInList);
    }

    /**
     * @param scoringPosition position you are scoring in. can be a call to {@link #getIdealScoringPosition()}
     * @param positionFrom position that you are going to the object from (usually odometry).
     * @return the next ideal game element to pickup
     */
    protected GameElement getIdealNextElementToPickup(int scoringPosition, Pose2d positionFrom) {
        //TODO: check distance to scoring position or change scoring position calculation to include a position coming from

        // poses that we will check for getNearest element latter
        ArrayList<GameElement> posesToCompareTo = new ArrayList<>(groundBucketSize);
        // the type of the game element that we need at the ideal scoring position based off of param
        GameElementType gameElementTypeNeeded = positionToGEType[scoringPosition];
        // range based foor loop for elements that are currently visible and on the ground
        for (int position0 = 0; position0 < groundBucketSize; position0++) {
            // if the current node we are reading is one that we need then. needed may be either, element on ground will never be
            if (gameElementTypeNeeded == GameElementType.EITHER || gameElementTypeNeeded == elementsOnGround[position0].getType()) {
                posesToCompareTo.add(elementsOnGround[position0]);
            }
        }

        // now use that list to check for the closest element to where we currently are (aka positionFrom)
        return getClosestElementTo(positionFrom, posesToCompareTo);
    }

    /**
     * gets the points you would get at a position
     * @param position0 the position of the game element
     * @return the points you would score at that position
     */
    protected int getScoreAt(final int position0) {
        if (isOccupied(position0)) {
            return 0;
        }
        final int position1 = positionToRowCheckerPosition1[position0];
        final int position2 = positionToRowCheckerPosition2[position0];

        final int rowBonus = positionToRowBonus[position0];
        final int linkBonus = ((isOccupied(position1) && isOccupied(position2)) ? 5 : 0);

        return rowBonus + linkBonus;
    }

    /**
     * @return whether we have the ranking point
     */
    protected boolean hasRankingPoint() {
        int numberFilled = 0; // number of rows that are filled for the ranking point
        // iterate through every row
        for (int i = 0; i < 27; i += 3) {
            if (rowIsCompleted(i)) {
                numberFilled++;
            }
        }
        return numberFilled > 4; // at least five links are scored
    }

    /**
     * @return the highest scoring position without worrying about ranking points
     */
    protected int getHighestScoringPosition() {
        int highest = 0;
        int indexOfHighest = 0;
        // 3 rows by 9 columns
        for (int i = 0; i < gridBucketSize; i++) {
            final int temp = getScoreAt(i);
            if (highest < temp) {
                highest = temp;
                indexOfHighest = i;
            }
        }
        return indexOfHighest;
    }


    /**
     * @return the ideal position on the grid to score including ranking point logic
     */
    public int getIdealScoringPosition() {
        // don't care about ranking points
        if (Constants.kEliminationRound || haveRankingPoint) {
            return getHighestScoringPosition();
        }
        // only care about ranking points
        else {
            // find the highest number of items in a row
            int highestNumInRow = 0;
            int indexOfHighestNumberOfItemRow = 0;
            for (int position0 = 0; position0 < 27; position0 += 3) {
                if (rowIsCompleted(position0)) {
                    continue;
                }
               final int position1 = positionToRowCheckerPosition1[position0];
               final int position2 = positionToRowCheckerPosition2[position0];
               final int numTakenUpInRow = (isOccupied(position0) ? 1 : 0) + (isOccupied(position1) ? 1 : 0) + (isOccupied(position2) ? 1 : 0);

               if (highestNumInRow < numTakenUpInRow) {
                   highestNumInRow = numTakenUpInRow;
                   indexOfHighestNumberOfItemRow = (!isOccupied(position0)) ? position0 :
                           (isOccupied(position1) ? position2 : position1);
               }
            }
            // if there are no items in any row
            if (highestNumInRow == 0) {
                return getHighestScoringPosition();
            }
            return indexOfHighestNumberOfItemRow;
        }
    }


}
