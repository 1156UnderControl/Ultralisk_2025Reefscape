package frc.Java_Is_UnderControl.Vision.Localization;

import java.util.Map;

import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import frc.Java_Is_UnderControl.Vision.Cameras.Data.AprilTagData;

public class MultiPoseEstimation {
    private AprilTagFieldLayout aprilTagFieldLayout;
    private Map<String, AprilTagData> cameraData;
    private AprilTagData bestTargetData;
    private int bestAprilTagID;
    private Pose3d robotPose;
    private String cameraName;

    public MultiPoseEstimation(Map<String, AprilTagData> cameraData) {
        try {
            this.aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        } catch (Exception e) {
            e.printStackTrace();
        }
        this.cameraData = cameraData;
        this.bestTargetData = null;
        this.bestAprilTagID = -1;
    }

    private void robotPoseCalc() {
        if (bestTargetData != null && aprilTagFieldLayout.getTagPose(bestAprilTagID).isPresent()) {
            this.robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                bestTargetData.getBestCameraToTarget(),
                aprilTagFieldLayout.getTagPose(bestAprilTagID).get(),
                bestTargetData.getCameraPosition()
            );
        }
    }

    private void selectBestTarget() {
        double closestDistance = Double.MAX_VALUE;

        for (Map.Entry<String, AprilTagData> entry : cameraData.entrySet()) {
            this.cameraName = entry.getKey();
            int tagID = entry.getValue().getAprilID();
            AprilTagData data = entry.getValue();

            if (data.getDistanceTarget() < closestDistance) {
                closestDistance = data.getDistanceTarget();
                bestTargetData = data;
                bestAprilTagID = tagID;
            }
        }
    }

    public Pose3d getRobotPose() {
        selectBestTarget();
        robotPoseCalc();
        return this.robotPose;
    }

    public String getCameraName(){
        return this.cameraName;
    }
}

