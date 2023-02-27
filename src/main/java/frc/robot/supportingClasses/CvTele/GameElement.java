package frc.robot.supportingClasses.CvTele;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

public class GameElement {
    // local constants
    protected static class Constants {
        public final static double heightGridDeadband = 0.1; // meters off the ground
        public static final BoundingBox scoringStationBlue = new BoundingBox(); //TODO: find values
        public static final BoundingBox scoringStationRed = new BoundingBox(); //TODO: find values
    }

    protected Pose3d position;
    protected boolean cone;
    protected static DriverStation.Alliance alliance = DriverStation.getAlliance();
    protected boolean isOnTheGrid;
    protected int xPositionOnGrid;
    protected int yPositionOnGrid;

    /**
     * Makes a new Game Element object
     *
     * @param position position of the game element in 3D coordinate space
     * @param isCone whether the game element is a cone or not (false for cube)
     */
    public GameElement(Pose3d position, boolean isCone) {
        this.position = position;
        this.cone = isCone;

        if (alliance == DriverStation.Alliance.Invalid) {
            alliance = DriverStation.getAlliance();
        }
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
        this(new Pose3d(position), isCone);
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

    protected boolean isOnTheCommunityGrid() {
        return Constants.scoringStationBlue.isInBox() || 
    }



}
