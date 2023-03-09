package frc.robot.supportingClasses.CvTele;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.ArrayList;
import java.util.List;

public class GameElement {
    // local constants
    protected static class Constants {
        public final static double heightGridDeadband = 0.1; // meters off the ground
        public static final BoundingBox scoringStationBlue = new BoundingBox(0, 0, 1.35, 5.5);
        public static final BoundingBox scoringStationRed = new BoundingBox(15, 0, 18, 5.5);
        public static final BoundingBox communityStationRed = new BoundingBox(0, 5.5, 0.3, 10);
        public static final BoundingBox communityStationBlue = new BoundingBox(16.2, 5.5, 17, 9);
        public static final double[] redCubableXpositions = new double[] {
                //TODO: find values
        }; // the x position that you can score on cubes

        public static final double[] redCubableYPositions = new double[] {
                //TODO: find values
        }; // the x position that you can score on cubes

        public static final double[] redConeableXpositions = new double[] {
                //TODO: find values
        }; // the x position that you can score on cones

        public static final double[] redConeableYPositions = new double[] {
                //TODO: find values
        }; // the x position that you can score on cones

        public static final double[] blueCubableXpositions = new double[] {
                //TODO: find values
        }; // the x position that you can score on cubes

        public static final double[] blueCubableYPositions = new double[] {
                //TODO: find values
        }; // the x position that you can score on cubes

        public static final double[] blueConeableXpositions = new double[] {
                //TODO: find values
        }; // the x position that you can score on cones

        public static final double[] blueConeableYPositions = new double[] {
                //TODO: find values
        }; // the x position that you can score on cones
    }

    // an enum for the type of the game element
    public enum GameElementType {
        CONE,
        CUBE,
        // either is a requisite type meant to be used as like it can be either cube or cone
        EITHER,
    }

    // the index will be the byte funny
    protected final static double[][] positionsMap = new double[][] {
            Constants.redCubableXpositions, Constants.redConeableXpositions,
            Constants.blueCubableXpositions, Constants.blueConeableXpositions,
            Constants.redCubableYPositions, Constants.redConeableYPositions,
            Constants.blueCubableYPositions, Constants.blueConeableYPositions
        };

    protected Pose3d position;
    protected final GameElementType type;

    protected static Alliance alliance = DriverStation.getAlliance();

    protected final Alliance gridGameElementIsOn; // which grid the game element is on, Invalid for not on one
    protected final int xPositionOnGrid; // x coordinate of the game element on the grid (0 -> 8) -1 for not on grid
    protected final int yPositionOnGrid; // y coordinate of the game element on the grid (0 -> 2) -1 for not on grid

    /**
     * Makes a new Game Element object
     *
     * @param position position of the game element in 3D coordinate space
     * @param type whether the game element is a cone or not (false for cube)
     */
    public GameElement(Pose3d position, GameElementType type) {
        this.position = position;
        this.type = type;

        if (alliance == Alliance.Invalid) {
            alliance = DriverStation.getAlliance();
        }

        gridGameElementIsOn = getWhatCommunityGridBoxIsObjectOn();

        if (gridGameElementIsOn == Alliance.Invalid) {
            xPositionOnGrid = -1;
            yPositionOnGrid = -1;
        }
        else {
            xPositionOnGrid = getXCoordOfGameElementOnGrid();
            yPositionOnGrid = getYCoordOfGameElementOnGrid();
        }
    }

    /**
     * Poses are in a different mathematical coordinate plane, so we have to switch the position we use
     * @return the y coordinate of the game element on the grid
     */
    protected int getYCoordOfGameElementOnGrid() {
        // the Y position is the column which is actually the Z coordinate to the bot
        return binSearch(position.getZ(), positionsMap[determineArray(1, gridGameElementIsOn, type)]);
    }

    /**
     * Poses are in a different mathematical coordinate plane, so we have to switch the position we use
     * @return the x coordinate of the game element on the grid
     */
    protected int getXCoordOfGameElementOnGrid() {
        // the X coordinate is a row which is actually the y position to the bot
        return binSearch(position.getY(), positionsMap[determineArray(0, gridGameElementIsOn, type)]);
    }

    /**
     * Wrapper for {@link #binSearch(double, int, int, double[])}. Performs a binary search of the whole passed in array
     *
     * @param key the key that is being searched
     * @param array the array to binary search
     * @return the value at the key
     */
    protected int binSearch(double key, double[] array) {
        return binSearch(key, 0, array.length, array);
    }

    //TODO: write a test case
    /**
     * Determines the array to use. by doing a mathematical binary search esc method
     *
     * @param xOrY (0 for x, 1 for y)
     * @param alliance red for 0, blue for 1
     * @param type cube for 0, cone for 1
     * @return the index of the array we need to use
     */
    protected int determineArray(int xOrY, Alliance alliance, GameElementType type) {
        // quick log base 2
        int i = Integer.numberOfLeadingZeros(positionsMap.length);
        int redOrBlue = alliance == Alliance.Red ? 0 : 1;
        int cubeOrCone = type == GameElementType.CONE ? 0 : 1;

        // believe it or not thi is a binary search lol
        return xOrY * (i <<= 2) + redOrBlue * (i <<= 2) + cubeOrCone * (i << 2);
    }

    /**
     * Makes a new Game Element object
     *
     * @param position position of the game element in 3D coordinate space
     * @param type the type of the game element (0 for cube, 1 for cone)
     */
    public GameElement(Pose3d position, int type) {
        this(position, type == 1);
    }

    /**
     * Makes a new Game Element object
     * @param position position of the game element in 3D coordinate space
     * @param type the type of the game element ("cone" for cone, "cube" for cube)
     */
    public GameElement(Pose3d position, String type) {
        this(position, type.equalsIgnoreCase("cone"));
    }

    /**
     * Makes a new Game Element object. Guarantees it is on the ground
     *
     * @param position position of the game element in 2D coordinate space
     * @param isCone whether the game element is a cone or not (false for cube)
     */
    public GameElement(Pose2d position, boolean isCone) {
        this(new Pose3d(position), isCone ? GameElementType.CONE : GameElementType.CUBE);
    }

    /**
     * Makes a new Game Element object. Guarantees it is on the ground
     *
     * @param position position of the game element in 3D coordinate space
     * @param isCone whether the game element is a cone or not (false for cube)
     */
    public GameElement(Pose3d position, boolean isCone) {
        this(position, isCone ? GameElementType.CONE : GameElementType.CUBE);
    }

    /**
     * Makes a new Game Element object. Guarantees it is on the ground
     *
     * @param position position of the game element in 2D coordinate space
     * @param type the type of the game element (0 for cube, 1 for cone)
     */
    public GameElement(Pose2d position, int type) {
        this(new Pose3d(position), type == 1);
    }

    /**
     * Makes a new Game Element object. Guarantees it is on the ground
     *
     * @param position position of the game element in 2D coordinate space
     * @param type the type of the game element ("cone" for cone, "cube" for cube)
     */
    public GameElement(Pose2d position, String type) {
        this(new Pose3d(position), type.equalsIgnoreCase("cone"));
    }

    /**
     * Gets whether the box is on a community grid and if it is which one
     * @return The alliance's community grid that the bot is on
     */
    protected Alliance getWhatCommunityGridBoxIsObjectOn() {
        // smh I am missing Rust's match feature
        if (Constants.scoringStationBlue.isInBox(position)) {
            return Alliance.Blue;
        }
        if (Constants.scoringStationRed.isInBox(position)) {
            return Alliance.Red;
        }
        return Alliance.Invalid;
    }


    /**
     * Performs a binary search. Helper method.
     * Returns the left hand bound which should be the index in the overall grid.
     *
     * @param key the key to search for
     * @param lowerBound the lowest element in the list or window you want to search through
     * @param upperBound the highest element in the list or window you want to search through
     * @param array the array you want to perform the search on
     * @return the position of the key in the array
     */
    protected int binSearch(double key, int lowerBound, int upperBound, double[] array) {
        char iterations = 0;

        // bin search go brrrrrrrr
        while (lowerBound != upperBound || iterations > Math.log(array.length) + 1) {
            int midPosition = (lowerBound + upperBound) / 2;
            // if the position is between the two options left
            if (lowerBound == upperBound - 1 && key > array[lowerBound] && key < array[upperBound]) {
                return lowerBound;
            }
            if (key < array[midPosition]) {
                upperBound = midPosition - 1;
            }
            else if (key > array[midPosition]) {
                lowerBound = midPosition + 1;
            }
            else {
                return midPosition;
            }
        }
        return -1;
    }

    /**
     * output:
     *      grid position: {x, y}
     *      position: 3D coordinates
     *      element type: CUBE or CONE
     *      alliance grid object is on: INVALID if on ground, Blue for blue side, Red for red side
     *
     * @return a string representation of this game element
     */
    public String toString() {
        return "grid position: {" + xPositionOnGrid + ", " + yPositionOnGrid + "}\n" + "position: " + position.getTranslation().toString()
                + "\n" + "element type: " + type.toString() + "alliance grid object is on: " + gridGameElementIsOn;
    }

    /**
     *  Wrapper method for {@link #closeTo(Pose3d)}
     *
     * @param other game element to compare to
     * @return whether you are close to another game element
     */
    public boolean closeTo(GameElement other) {
        return closeTo(other.getPosition3D());
    }

    /**
     * Whether this is close to another position.
     * returns true if close enough to another object to be the same object.
     *
     * @param otherPosition position to compare to
     * @return whether you are close to another position
     */
    private boolean closeTo(Pose3d otherPosition) {
        return (Math.abs(position.getX() - otherPosition.getX()) < 0.1 &&
                (Math.abs(position.getY() - otherPosition.getY()) < 0.1) &&
                Math.abs(position.getZ() - otherPosition.getZ()) < 0.1);
    }

    /**
     * @param o other object to compare this to
     * @return whether the objects are equal or not
     */
    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        GameElement that = (GameElement) o;

        return type == that.type && gridGameElementIsOn == that.gridGameElementIsOn &&
                xPositionOnGrid == that.xPositionOnGrid &&
                yPositionOnGrid == that.yPositionOnGrid &&
                this.closeTo(that);
    }

    /**
     * @return Position in 3D coordinate space
     */
    public Pose3d getPosition3D() {
        return position;
    }

    /**
     * @return Position in 2D coordinate space
     */
    public Pose2d getPosition2D() {
        return position.toPose2d();
    }

    /**
     * @return the position on the gird if it were all in one array
     */
    public int getIndexOnGrid() {
        return xPositionOnGrid + yPositionOnGrid * 3;
    }

    /**
     * @return the x position on the grid (row)
     */
    public int getXIndexOnGrid() {
        return xPositionOnGrid;
    }

    /**
     * @return the y position on the grid (column)
     */
    public int getYIndexOnGrid() {
        return yPositionOnGrid;
    }

    /**
     * @return if it is a cube
     */
    public boolean isCube() {
        return type == GameElementType.CUBE;
    }

    /**
     * @return if it is a cone
     */
    public boolean isCone() {
        return type == GameElementType.CONE;
    }

    /**
     * @return the y position of the bot
     */
    public double getY() {
        return position.getY();
    }

    /**
     * @return the x position of the bot
     */
    public double getX() {
        return position.getX();
    }

    /**
     * @return the z position of the bot
     */
    public double getZ() {
        return position.getZ();
    }

    /**
     * @param otherPoint point that the transformation is made from in relation to this. usually odometry
     * @return the transformation to the position
     */
    public Transform3d getTranslationToPose(Pose3d otherPoint) {
        return position.minus(otherPoint);
    }

    /**
     *  Overload of {@link #getTranslationToPose(Pose3d)}
     *
     * @param otherPoint point that the transformation is made from in relation to this. usually odometry
     * @return the transformation to the position
     */
    public Transform2d getTranslationToPose(Pose2d otherPoint) {
        return position.toPose2d().minus(otherPoint);
    }

    /**
     * @param poses list of poses that you are searching the nearest point to
     * @return the nearest pose2D from the passed in list of poses
     */
    public Pose2d getNearestPoseTo(List<Pose2d> poses) {
        return position.toPose2d().nearest(poses);
    }

    /**
     * @param otherPose the other position you are comparing this to
     * @return the distance to another pose as a double
     */
    public double getDistanceToPosition(Pose2d otherPose) {
        return Math.hypot(position.getX() - otherPose.getX(), position.getY() - otherPose.getY());
    }

    /**
     * Loose list of types (maybe out of date): CONE, CUBE, EITHER
     * @return the game element type for options see: {@link GameElementType}
     */
    public GameElementType getType() {
        return this.type;
    }
}
