package frc.Java_Is_UnderControl.Vision.Cameras.Data;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomPose3dLogger;

public class AprilTagData {

    private String cameraName;
    private int aprilID;
    private double aprilYaw;
    private double aprilPitch;
    private double aprilSkew;
    private double aprilArea;
    private double distanceToApril;
    private Transform3d camToRobot;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private Transform3d bestCameraToApril;

    AprilTagData aprilData;
    CustomDoubleLogger pitchLog;
    CustomDoubleLogger yawLog;
    CustomDoubleLogger skewLog;
    CustomDoubleLogger areaLog;
    CustomDoubleLogger distanceToAprilLog;
    CustomPose3dLogger aprilPoseLog;
    Optional<Pose3d> robotPose;
    int numberOfTargetsUsed;

    public AprilTagData(String cameraName,
            int aprilID,
            double aprilYaw,
            double aprilPitch,
            double aprilSkew,
            double aprilArea,
            double distanceToApril,
            Transform3d camToRobot,
            Optional<Pose3d> robotPose,
            int numbersOfTargetsUsed) {

        this.aprilID = aprilID;
        this.aprilYaw = aprilYaw;
        this.aprilPitch = aprilPitch;
        this.aprilSkew = aprilSkew;
        this.aprilArea = aprilArea;
        this.distanceToApril = distanceToApril;
        this.camToRobot = camToRobot;
        this.aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    }

    public int getAprilID() {
        return this.aprilID;
    }

    public String getCameraName() {
        return this.cameraName;
    }

    public double getPitch() {
        return this.aprilPitch;
    }

    public double getYaw() {
        return this.aprilYaw;
    }

    public double getSkew() {
        return this.aprilSkew;
    }

    public double getArea() {
        return this.aprilArea;
    }

    public double getDistanceTarget() {
        return this.distanceToApril;
    }

    public Transform3d getCameraPosition() {
        return this.camToRobot;
    }

    public Optional<Pose3d> getOptional(){
        return this.robotPose;
    }

    public int getNumberOfTargetsUsed(){
        return this.numberOfTargetsUsed;
    }

    public boolean hasAprilTags() {
        if (this.getYaw() == 0 &&
                this.getPitch() == 0 &&
                this.getSkew() == 0 &&
                this.getArea() == 0) {
            return false;
        } else {
            return true;
        }
    }

    public Pose3d getTargetPose() {
        return this.aprilTagFieldLayout.getTagPose(this.aprilID)
                .orElseThrow(() -> new IllegalAccessError("Pose Not Found"));
    }

    public Transform3d getBestCameraToTarget() {
        return this.bestCameraToApril;
    }

    private void setLogs(){
        this.pitchLog = new CustomDoubleLogger("/Vision/" + this.cameraName + "/" + (this.hasTargets() ? this.aprilData.getAprilID() + "/pitch" : "No_Tags_Seen"));
        this.yawLog = new CustomDoubleLogger("/Vision/" + this.cameraName + "/" + (this.hasTargets() ? this.aprilData.getAprilID() + "/yaw" : "No_Tags_Seen"));
        this.skewLog = new CustomDoubleLogger("/Vision/" + this.cameraName + "/" + (this.hasTargets() ? this.aprilData.getAprilID() + "/skew" : "No_Tags_Seen"));
        this.areaLog = new CustomDoubleLogger("/Vision/" + this.cameraName + "/" + (this.hasTargets() ? this.aprilData.getAprilID() + "/area" : "No_Tags_Seen"));
        this.distanceToAprilLog = new CustomDoubleLogger("/Vision/" + aprilData.getCameraName() + "/" + (this.hasTargets() ? this.aprilData.getAprilID() + "/distanceToApril" : "No_Tags_Seen"));
        this.aprilPoseLog = new CustomPose3dLogger("/Vision/" + aprilData.getCameraName() + "/" + (this.hasTargets() ? this.aprilData.getAprilID() + "/aprilPose" : "No_Tags_Seen"));
    }

    public void updateLogs(){
        if(this.hasTargets()){
            this.setLogs();
            this.pitchLog.append(this.aprilData.getPitch());
            this.yawLog.append(this.aprilData.getYaw());
            this.skewLog.append(this.aprilData.getSkew());
            this.areaLog.append(this.aprilData.getArea());
            this.distanceToAprilLog.append(this.aprilData.getDistanceTarget());
            this.aprilPoseLog.appendDegrees(this.aprilData.getTargetPose());
        }
    }

    public boolean hasTargets(){
        return this.aprilData.hasTargets();
    }
}