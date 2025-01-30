package frc.Java_Is_UnderControl.Vision.Cameras.Types.PhotonVision;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomPose3dLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomTransform3dLogger;
import frc.Java_Is_UnderControl.Vision.Cameras.Data.ObjectData;
import frc.Java_Is_UnderControl.Vision.Cameras.Types.Interfaces.ICameraObject;
import frc.Java_Is_UnderControl.Vision.Cameras.Types.Interfaces.ICameraOdometry;
import frc.Java_Is_UnderControl.Vision.Object_Detection.ObjectDetection;
import frc.Java_Is_UnderControl.Vision.Object_Detection.ObjectDetectionCamera;
import frc.Java_Is_UnderControl.Vision.Object_Detection.ObjectPoseEstimationRobotOriented;
import frc.Java_Is_UnderControl.Vision.Odometry.PoseEstimation;

public class PhotonVisionCamera implements ICameraOdometry, ICameraObject {

  private String cameraName;
  private String objectName;
  private double maxTx;
  private double maxTy;
  private Pose2d robotPose;
  private PhotonCamera camera;
  private Rotation2d robotRotation;
  private Transform3d camToRobot;

  private ObjectData objectData;
  private ObjectDetection objectDetection;
  private ObjectDetectionCamera objectDetectionCamera;
  private ObjectPoseEstimationRobotOriented objectPose;
  private AprilTagFieldLayout aprilTagFieldLayout;

  private CustomDoubleLogger pitchLog;
  private CustomDoubleLogger yawLog;
  private CustomDoubleLogger areaLog;
  private CustomDoubleLogger distanceToAprilLog;
  private CustomPose3dLogger aprilPoseLog;
  private CustomTransform3dLogger camToRobotLog;

  private double pitch = 0;

  private double yaw = 0;

  private double area = 0;

  private double distanceToTarget = 0;

  public PhotonVisionCamera(String cameraName, String objectName, double maxTx, double maxTy, Pose2d robotPose,
      Translation3d translation, Rotation3d rotation) {
    this.camera = new PhotonCamera(cameraName);
    this.cameraName = cameraName;
    this.objectName = objectName;
    this.maxTx = maxTx;
    this.maxTy = maxTy;
    this.robotPose = robotPose;
    this.aprilTagFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
    this.setCamToRobot(translation, rotation);
  }

  public PhotonVisionCamera(String cameraName, Translation3d translation, Rotation3d rotation) {
    this(cameraName, null, Double.NaN, Double.NaN, null, translation, rotation);
  }

  private void setCamToRobot(Translation3d translation, Rotation3d rotation) {
    this.camToRobot = new Transform3d(translation, rotation);
  }

  public String getCameraName() {
    return this.cameraName;
  }

  public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }

  private double getDistanceAprilTag(PhotonTrackedTarget photonTrackedTarget) {
    var result = camera.getLatestResult();
    double distance = 0;
    if (result.hasTargets()) {
      PhotonTrackedTarget target = photonTrackedTarget;
      distance = PhotonUtils.calculateDistanceToTargetMeters(this.camToRobot.getY(),
          aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().getZ(),
          Units.degreesToRadians(this.camToRobot.getX()),
          Units.degreesToRadians(target.getPitch()));
      return distance;
    } else {
      return distance;
    }
  }

  @Override
  public Optional<PoseEstimation> getRobotPose() {
    PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camToRobot);
    var robotPose = poseEstimator.update(this.getLatestResult());
    if (robotPose.isPresent()) {
      return Optional.of(new PoseEstimation(robotPose.get().estimatedPose,
          robotPose.get().timestampSeconds,
          robotPose.get().targetsUsed.size(),
          this.getDistanceAprilTag(this.getLatestResult().getBestTarget())));
    } else {
      return Optional.empty();
    }
  }

  @Override
  public void setAprilTagData() {
    PhotonPipelineResult result = camera.getLatestResult();
    if (result.hasTargets()) {
      this.setPitch(result.getBestTarget().getPitch());
      this.setYaw(result.getBestTarget().getYaw());
      this.setArea(this.getDistanceAprilTag(result.getBestTarget()));
      this.setDistanceTarget(this.getDistanceAprilTag(result.getBestTarget()));
    }
  }

  private void setPitch(double pitch) {
    this.pitch = pitch;
    this.updateLogsAprilTag();
  }

  private void setYaw(double yaw) {
    this.yaw = yaw;
    this.updateLogsAprilTag();
  }

  private void setArea(double area) {
    this.area = area;
    this.updateLogsAprilTag();
  }

  private void setDistanceTarget(double distanceToTarget) {
    this.distanceToTarget = distanceToTarget;
    this.updateLogsAprilTag();
  }

  @Override
  public void updateLogsAprilTag() {
    this.pitchLog.append(this.pitch);
    this.yawLog.append(this.yaw);
    this.areaLog.append(this.area);
    this.camToRobotLog.appendDegrees(this.camToRobot);
  }

  private void setObjectName(String objectName) {
    this.objectName = objectName;
  }

  private void setObjectPose() {
    if (this.objectName == null && this.maxTx == Double.NaN && this.maxTy == Double.NaN && this.objectPose == null) {
      System.out.println("You used the wrong PhotonVisionCamera constructor method!");
      return;
    }
    this.setObjectData();
    this.objectPose = new ObjectPoseEstimationRobotOriented(this.objectDetection, this.objectDetectionCamera);
  }

  private double getDistanceObject(PhotonTrackedTarget photonTrackedTarget) {
    var result = camera.getLatestResult();
    double distance = 0;
    if (result.hasTargets()) {
      PhotonTrackedTarget target = photonTrackedTarget;
      distance = PhotonUtils.calculateDistanceToTargetMeters(this.camToRobot.getY(),
          this.getObjectPose().getY(),
          Units.degreesToRadians(this.camToRobot.getX()),
          Units.degreesToRadians(target.getPitch()));
      return distance;
    }
    return distance;
  }

  private void setObjectData() {
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      for (PhotonTrackedTarget target : result.getTargets()) {
        this.objectDetection = new ObjectDetection(target.getYaw(), target.getPitch(),
            (this.objectName != null ? this.objectName : "object"));
        this.objectData = new ObjectData(this.cameraName, this.objectName, objectDetection,
            this.getDistanceObject(target), camToRobot);
      }
    }
  }

  @Override
  public Pose2d getObjectPose() {
    this.setObjectName(this.objectName);
    this.setObjectData();
    this.setObjectPose();
    Pose2d objectPose2d = this.objectPose.getObjectPoseFromRobotCenter();
    return objectPose.getObjectPoseInField(this.robotPose, objectPose2d);
  }

  @Override
  public ObjectData getObjectData() {
    this.setObjectData();
    return this.objectData;
  }

  @Override
  public void updateLogsObject() {
    this.setObjectData();
  }

  public void setRobotRotation(Rotation2d rotation) {
    this.robotRotation = rotation;
  }

  public Rotation2d getRobotRotation() {
    return this.robotRotation;
  }

  public int getNumberOfTargetsDetected() {
    var result = camera.getLatestResult();
    return result.getTargets().size();
  }
}
