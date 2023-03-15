package frc.robot.supportingClasses.CvTele;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.List;

public class GameElement {
    // local constants
    protected static class Constants {
        // public final static double heightGridDeadband = 0.1; // meters off the ground

        // the bounding box that dictates the location of the blue scoring station in 3D space on the field
        public static final BoundingBox scoringStationBlue = new BoundingBox(0, 0, 1.43, 5.5);
        // the bounding box that dictates the location of the red scoring station in 3D space on the field
        public static final BoundingBox scoringStationRed = new BoundingBox(14.55, 0, 18, 5.5);
        // the bounding box that dictates the location of the red community station in 3D space on the field
        public static final BoundingBox communityStationRed = new BoundingBox(0, 5.5, 0.3, 10);
        // the bounding box that dictates the location of the blue community station in 3D space on the field
        public static final BoundingBox communityStationBlue = new BoundingBox(16.2, 5.5, 17, 9);

        public static final double[] yPositionsConesForColumnBounds = new double[] {
            // 0    1    2
            -1, 0.4, 0.9, 1000
        }; // the y position that you can score on cones.

        public static final double[] yPositionsCubesForColumnBounds = new double[] {
            // 0    1     2
            -1, 0.5, 0.83, 1000
        }; // the y position that you can score on cubes

        public static final double[] xPositionsForRowBounds = new double[] {
            //0       1       2       3       4       5       6       7       8
            0, 0.7875, 1.3465, 1.9055, 2.4645, 3.0235, 3.5825, 4.1415, 4.7005, 5.5
        };

        public static final double kXMultiplierForGrid = 1.8; // the x multiplier for getting the grid position: see ProjectRoot/Docs/RangeMultipliers.pdf
        public static final double kYCubeMultiplierForGrid = 2.5; // the y multiplier for cubes for getting the grid position: see same as above
        public static final double kYConeMultiplierForGrid = 2.5; // the y multiplier for cones for getting the grid position: see same as above
    }

    // an enum for the type of the game element
    public enum GameElementType {
        CONE,
        CUBE,
        // "Either" is a requisite type meant to be used as: can be either a cube or a cone
        EITHER,
    }

    protected Pose3d position; // the position of the bot on the field in 3D coordinate space
    protected final GameElementType type; // the type of the game element: cube or cone

    protected final Alliance stationGameElementIsIn; // which alliance station the game element is in
    protected final boolean isOnGround; // is the game element on the ground
    protected final boolean inCommunityStation; // is the game element in a community station
    protected final boolean onGrid; // is the game element on a grid

    protected final Alliance gridGameElementIsOn; // which grid the game element is on, Invalid for not on one
    protected final int xPositionOnGrid; // x coordinate of the game element on the grid (0 -> 8) -1 for not on grid
    protected final int yPositionOnGrid; // y coordinate of the game element on the grid (0 -> 2) -1 for not on grid

    /**
     * Makes a new Game Element object.
     *
     * @param position position of the game element in 3D coordinate space
     * @param type whether the game element is a cone or not (false for cube)
     */
    public GameElement(Pose3d position, GameElementType type) {
        this.position = position;
        this.type = type;

        gridGameElementIsOn = getWhatCommunityGridBoxIsObjectOn();
        stationGameElementIsIn = getStationGameElementIsIn();

        inCommunityStation = stationGameElementIsIn != Alliance.Invalid;
        onGrid = gridGameElementIsOn != Alliance.Invalid;
        isOnGround = !(inCommunityStation || onGrid);

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
     * <p>gets the station that the game element is in using {@link Constants#scoringStationBlue} and {@link Constants#scoringStationRed}.
     *  Both of these objects are Bounding boxes that are representative of the region they take up on the field.</p>
     * <b> Needs to be run after {@link #position} is initialized.</b>
     * @return the community station that the game element is in.
     */
    protected Alliance getStationGameElementIsIn() {
        if (Constants.scoringStationBlue.isInBox(position)) {
            return Alliance.Blue;
        }
        else if (Constants.scoringStationRed.isInBox(position)) {
            return Alliance.Red;
        }
        else {
            return Alliance.Invalid;
        }
    }

    /**
     * Poses are in a different mathematical coordinate plane, so we have to switch the position we use
     * @return the y coordinate of the game element on the grid
     */
    protected int getYCoordOfGameElementOnGrid() {
        int result = (int) (position.getZ() * (
                (type == GameElementType.CUBE) ? Constants.kYConeMultiplierForGrid : Constants.kYCubeMultiplierForGrid)
        ); // z cause that's height

        if (result < 0) {
            return 0;
        }
        else if (result > 2) {
            return 2;
        }

        return result;
    }

    /**
     * @return the x coordinate of the game element on the grid
     */
    protected int getXCoordOfGameElementOnGrid() {
        int result = (int) (position.getY() * Constants.kXMultiplierForGrid); // y is x coord post transform
        if (result < 0) {
            return 0;
        }
        else if (result > 8) {
            return 8;
        }
        return result;
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

    /**
     * @return if the game element is on the ground
     */
    public boolean isOnGround() {
        return isOnGround;
    }

    /**
     * @return if "this" is on the grid or not
     */
    public boolean isOnGrid() {
        return onGrid;
    }

    /**
     * @return if "this" is in the community station
     */
    public boolean isInCommunityStation() {
        return inCommunityStation;
    }


}
