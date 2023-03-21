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
import frc.robot.supportingClasses.VisionMedianFilter;
import frc.robot.supportingClasses.OdoPosition;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;

public class Limelight {

    protected PhotonCamera camera;

    protected final GenericEntry nXCameraToTarget;
    protected final GenericEntry nYCameraToTarget;
    protected final GenericEntry nsuccessfulImageReads;

    private static ShuffleboardTab tab = Shuffleboard.getTab("PhotonCamera");
    AprilTagFieldLayout aprilTagFieldLayout;
    VisionMedianFilter filter;
    int successfulUpdates = 0;

    protected double lastReadTime = 0;

    public Limelight() {
        camera = new PhotonCamera("OV5647");

        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch(IOException e){
            DriverStation.reportError("error loading field position file", false);
        }

        filter = new VisionMedianFilter(Camera.kMedianFilterWindowSize);

        if (Constants.debugMode) {
            SmartDashboard.putData(filter);
        }

        nXCameraToTarget = tab.add("X Camera to target", 0).getEntry();
        nYCameraToTarget = tab.add("Y camera to target", 0).getEntry();

        nsuccessfulImageReads = tab.add("Successful image reads", 0).getEntry();
    }

    /**
     * method to write generic entry changes
     * TODO: Remove before comp
     */
    public void outputToShuffleBoard(){
        PhotonPipelineResult result = camera.getLatestResult();

        if (!result.hasTargets()) {
            return;
        }
        PhotonTrackedTarget target = result.getBestTarget();

        Transform3d transformation = target.getBestCameraToTarget();

        Translation2d translation = transformation.getTranslation().toTranslation2d();
        nXCameraToTarget.setDouble(translation.getX());
        nYCameraToTarget.setDouble(translation.getY());

        nsuccessfulImageReads.setInteger(successfulUpdates);
    }

    /**
     * @return the x position of the camera relative to the april tag
     */
    public double getX(){
        PhotonPipelineResult result = camera.getLatestResult();
        if (!result.hasTargets()) {
            return Double.MAX_VALUE;
        }
        PhotonTrackedTarget target = result.getBestTarget();

        Transform3d transformation = target.getBestCameraToTarget();

        Translation2d translation = transformation.getTranslation().toTranslation2d();
        return translation.getX();
    }


    /**
     * Calculates the position of the bot relative to an april tag.
     * That calculation is then given to {@link VisionMedianFilter}.
     * The command will return null if there is no new information or if there are no targets in frame.
     * This will add all the targets that are currently visible to the {@link VisionMedianFilter}.
     *
     * @return the filtered camera position
     */
    public OdoPosition calculate() {
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

            // the position of the bot relative to the april tag
            Pose3d position = PhotonUtils.estimateFieldToRobotAprilTag(
                    bestCameraToTarget,
                    aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(),
                    cameraToCenterOfBot);

            /* updates the best value that we will return on the last iteration,
              also passes the read position into the {@link VisionMedianFilter)
             */
            best = filter.getOdoPose(new OdoPosition(position.toPose2d(), result.getTimestampSeconds()));;
        }
        // returns the last filtered value that we checked in the above for loop
        return best;
    }


    /**
     * @return the number of successful images we have read
     */
    public int getNumberOfSuccesses() {
        return successfulUpdates;
    }

}

