package frc.robot.sensors;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Newman_Constants.Constants;
import frc.robot.Newman_Constants.Constants.Camera;
import frc.robot.supportingClasses.Vision.VisionMedianFilter;
import frc.robot.supportingClasses.Vision.OdoPosition;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.Optional;

public class Limelight {

    protected PhotonCamera camera;

    private static final ShuffleboardTab tab = Shuffleboard.getTab("PhotonCamera");
    AprilTagFieldLayout aprilTagFieldLayout;
    VisionMedianFilter filter;
    int successfulUpdates = 0;

    GenericEntry n_yaw;
    GenericEntry n_pitch;

    protected double lastReadTime = 0;

    /**
     * Constructs a new Limelight object.
     * The limelight object will be full of null values if Constants.useAprilTags is false.
     */
    public Limelight() {
        if (Constants.useAprilTags) {
            camera = new PhotonCamera("OV5647");
            try {
                aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            } catch (IOException e) {
                DriverStation.reportError("error loading field position file", false);
            }

            filter = new VisionMedianFilter(Camera.kMedianFilterWindowSize);

            if (Constants.debugMode) {
                SmartDashboard.putData(filter);
            }

            n_yaw = Shuffleboard.getTab("PhotonVision").add("Yaw", 0).getEntry();
            n_pitch = Shuffleboard.getTab("PhotonVision").add("Pitch", 0).getEntry();
        }   
    }

    /**
     * Calculates the position of the bot relative to an april tag.
     * That calculation is then given to {@link VisionMedianFilter}.
     * The command will return null if there is no new information or if there are no targets in frame.
     * This will add all the targets that are currently visible to the {@link VisionMedianFilter}.
     * If {@link Constants#useAprilTags} is false, this will return null.
     *
     * @return the filtered camera position
     */
    public OdoPosition calculate() {
        if (!Constants.useAprilTags || !camera.isConnected()) {
            return null;
        }
        // the most recent result as read by the camera
        PhotonPipelineResult result = camera.getLatestResult();

        // if there is no new results or if there are no targets on the screen
        if (result.getTimestampSeconds() == lastReadTime || !result.hasTargets()) {
            return null;
        }

        // increment the amount of successful updates we have read
        successfulUpdates++;
        lastReadTime = result.getTimestampSeconds();

        // default value for what we will return
        OdoPosition best = null;

        // for each target that is currently on the screen
        for (PhotonTrackedTarget target : result.getTargets()) {
            // x is forward, y is left, z is up
            Transform3d bestCameraToTarget = target.getBestCameraToTarget();


            // the matrix transformation for the camera to the center of the bot
            Transform3d cameraToCenterOfBot = new Transform3d(
                    new Translation3d(Camera.xPos, Camera.yPos, Camera.zPos),
                    new Rotation3d(Camera.roll, Camera.pitch, Camera.yaw));

            Optional<Pose3d> pose3d = aprilTagFieldLayout.getTagPose(target.getFiducialId());
            if (pose3d.isEmpty()) {
                return null;
            }

            // the position of the bot relative to the april tag
            Pose3d position = PhotonUtils.estimateFieldToRobotAprilTag(
                    bestCameraToTarget,
                    pose3d.get(),
                    cameraToCenterOfBot);

            /* updates the best value that we will return on the last iteration,
              also passes the read position into the {@link VisionMedianFilter)
             */
            best = filter.getOdoPose(new OdoPosition(position.toPose2d(), result.getTimestampSeconds()));;
        }
        // returns the last filtered value that we checked in the above for loop
        return best;
    }

}

