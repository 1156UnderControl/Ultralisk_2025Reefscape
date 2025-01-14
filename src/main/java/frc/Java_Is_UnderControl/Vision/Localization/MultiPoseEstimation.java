package frc.Java_Is_UnderControl.Vision.Localization;

import java.util.Map;

import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import frc.Java_Is_UnderControl.Vision.Cameras.TargetData;

public class MultiPoseEstimation {
    private AprilTagFieldLayout aprilTagFieldLayout;
    private Map<String, Map<Integer, TargetData>> cameraData;
    private TargetData bestTargetData;
    private int bestAprilTagID;
    private Pose3d robotPose;

    public MultiPoseEstimation(Map<String, Map<Integer, TargetData>> cameraData) {
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

        for (String cameraName : cameraData.keySet()) {
            Map<Integer, TargetData> targets = cameraData.get(cameraName);
            for (Map.Entry<Integer, TargetData> entry : targets.entrySet()) {
                int tagID = entry.getKey();
                TargetData data = entry.getValue();

                if (data.getDistanceTarget() < closestDistance) {
                    closestDistance = data.getDistanceTarget();
                    bestTargetData = data;
                    bestAprilTagID = tagID;
                }
            }
        }
    }

    public Pose3d getRobotPose() {
        selectBestTarget();
        robotPoseCalc();
        return this.robotPose;
    }
}

