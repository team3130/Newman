package frc.robot.sensors;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Newman_Constants.Constants;
import frc.robot.Newman_Constants.Constants.Camera;
import frc.robot.supportingClasses.KugelMedianFilter;
import frc.robot.supportingClasses.OdoPosition;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.lang.annotation.Target;
import java.util.ArrayList;

public class Limelight {

    PhotonCamera camera;

    protected final GenericEntry nXCameraToTarget;
    protected final GenericEntry nYCameraToTarget;
    protected final GenericEntry nsuccessfulImageReads;

    private static ShuffleboardTab tab = Shuffleboard.getTab("PhotonCamera");
    AprilTagFieldLayout aprilTagFieldLayout;
    KugelMedianFilter filter;
    int successfulUpdates = 0;

    protected double lastReadTime = 0;

    public Limelight() {
        camera = new PhotonCamera("OV5647");

        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch(IOException e){
            DriverStation.reportError("error loading field position file", false);
        }

        filter = new KugelMedianFilter(Camera.kMedianFilterWindowSize);

        if (Constants.debugMode) {
            SmartDashboard.putData(filter);
        }

        nXCameraToTarget = tab.add("X Camera to target", 0).getEntry();
        nYCameraToTarget = tab.add("Y camera to target", 0).getEntry();

        nsuccessfulImageReads = tab.add("Successful image reads", 0).getEntry();
    }

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

    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

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


    public OdoPosition calculateCameraPosition() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.getTimestampSeconds() == lastReadTime || !result.hasTargets()) {
            return null;
        }

        successfulUpdates++;
        lastReadTime = result.getTimestampSeconds();

        OdoPosition best = null;

        for (PhotonTrackedTarget target : result.getTargets()) {
            // x is forward, y is left, z is up
            Transform3d bestCameraToTarget = target.getBestCameraToTarget();


            // the matrix transformation for the camera to the center of the bot
            Transform3d cameraToCenterOfBot = new Transform3d(
                    new Translation3d(Camera.xPos, Camera.yPos, Camera.zPos),
                    new Rotation3d(Camera.roll, Camera.pitch, Camera.yaw));

            Pose3d position = PhotonUtils.estimateFieldToRobotAprilTag(
                    bestCameraToTarget,
                    aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(),
                    cameraToCenterOfBot);

            best = filter.getOdoPose(new OdoPosition(position.toPose2d(), result.getTimestampSeconds()));;
        }
        return best;
    }


    public int getNumberOfSuccesses() {
        return successfulUpdates;
    }

}

