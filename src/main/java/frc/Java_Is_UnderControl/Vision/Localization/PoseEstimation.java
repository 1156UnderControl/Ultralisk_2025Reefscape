package frc.Java_Is_UnderControl.Vision.Localization;

import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import frc.Java_Is_UnderControl.Vision.Cameras.Data.AprilData;

public class PoseEstimation {

    private AprilTagFieldLayout aprilTagFieldLayout;
    private AprilData targetData;
    private AprilData bestTargetData;
    private int bestAprilTagID;
    private Pose3d robotPose;

    public PoseEstimation(AprilData targetData){
        try {
            this.aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        } catch (Exception e) {
            e.printStackTrace();
        }

        this.targetData = targetData;
        this.bestTargetData = null;
    }

    private void robotPoseCalc() {
        if (bestTargetData != null && aprilTagFieldLayout.getTagPose(bestAprilTagID).isPresent()) {
            this.robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                bestTargetData.getBestCameraToTarget(),
                bestTargetData.getTargetPose(),
                bestTargetData.getCameraPosition()
            );
        }
    }

    private void selectBestTarget() {
        double closestDistance = Double.MAX_VALUE;
                int tagID = this.targetData.getAprilID();
                AprilData data = this.targetData;

                if (data.getDistanceTarget() < closestDistance) {
                    closestDistance = data.getDistanceTarget();
                    bestTargetData = data;
                    bestAprilTagID = tagID;
                }
    }

    public Pose3d getRobotPose() {
        selectBestTarget();
        robotPoseCalc();
        return this.robotPose;
    }


}