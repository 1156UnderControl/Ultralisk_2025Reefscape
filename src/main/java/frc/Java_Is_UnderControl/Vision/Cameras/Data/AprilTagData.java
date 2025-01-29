package frc.Java_Is_UnderControl.Vision.Cameras.Data;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomIntegerLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomPose3dLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomTransform3dLogger;

public class AprilTagData {

  private String cameraName;
  private int aprilID;
  private double aprilYaw;
  private double aprilPitch;
  private double aprilArea;
  private double distanceToApril;
  private Transform3d camToRobot;
  private AprilTagFieldLayout aprilTagFieldLayout;
  private Transform3d bestCameraToApril;

  private CustomDoubleLogger pitchLog;
  private CustomDoubleLogger yawLog;
  private CustomDoubleLogger areaLog;
  private CustomDoubleLogger distanceToAprilLog;
  private CustomPose3dLogger aprilPoseLog;
  private CustomTransform3dLogger camToRobotLog;
  private CustomIntegerLogger aprilIDLog;

  public AprilTagData(String cameraName,
      int aprilID,
      double aprilYaw,
      double aprilPitch,
      double aprilArea,
      double distanceToApril,
      Transform3d camToRobot) {

    this.aprilID = aprilID;
    this.aprilYaw = aprilYaw;
    this.aprilPitch = aprilPitch;
    this.aprilArea = aprilArea;
    this.distanceToApril = distanceToApril;
    this.camToRobot = camToRobot;
    this.aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    this.setLogs();
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

  public double getArea() {
    return this.aprilArea;
  }

  public double getDistanceTarget() {
    return this.distanceToApril;
  }

  public Transform3d getCameraPosition() {
    return this.camToRobot;
  }

  public Pose3d getTargetPose() {
    return this.aprilTagFieldLayout.getTagPose(this.aprilID)
        .orElseThrow(() -> new IllegalAccessError("Pose Not Found"));
  }

  public Transform3d getBestCameraToTarget() {
    return this.bestCameraToApril;
  }

  public void setPitch(double pitch) {
    this.aprilPitch = pitch;
  }

  public void setYaw(double yaw) {
    this.aprilYaw = yaw;
  }

  public void setArea(double area) {
    this.aprilArea = area;
  }

  public void setDistanceTarget(double distanceToApril) {
    this.distanceToApril = distanceToApril;
  }

  public void setBestCameraToTarget(Transform3d bestcameraToTarget) {
    this.bestCameraToApril = bestcameraToTarget;
  }

  private void setLogs() {
    this.pitchLog = new CustomDoubleLogger(
        "/Vision/" + this.cameraName + "/" + this.aprilID + "/pitch");
    this.yawLog = new CustomDoubleLogger(
        "/Vision/" + this.cameraName + "/" + this.aprilID + "/yaw");
    this.areaLog = new CustomDoubleLogger(
        "/Vision/" + this.cameraName + "/" + this.aprilID + "/area");
    this.distanceToAprilLog = new CustomDoubleLogger(
        "/Vision/" + this.cameraName + "/" + this.aprilID + "/distanceToApril");
    this.aprilPoseLog = new CustomPose3dLogger(
        "/Vision/" + this.cameraName + "/" + this.aprilID + "/aprilPose");
    this.camToRobotLog = new CustomTransform3dLogger("/Vision/" + this.cameraName
        + "/" + "cameraPosition");
    this.aprilIDLog = new CustomIntegerLogger("/Vision/" + this.cameraName + "/"
        + this.aprilID);
  }

  public void updateLogs() {
    this.pitchLog.append(this.getPitch());
    this.yawLog.append(this.getYaw());
    this.areaLog.append(this.getArea());
    this.distanceToAprilLog.append(this.getDistanceTarget());
    this.aprilPoseLog.appendDegrees(this.getTargetPose());
    this.camToRobotLog.appendDegrees(this.camToRobot);
  }
}
