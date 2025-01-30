package frc.Java_Is_UnderControl.Vision.Cameras.Types.Limelight;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomIntegerLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomPose3dLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomStringLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomTransform3dLogger;
import frc.Java_Is_UnderControl.Vision.Cameras.Data.ObjectData;
import frc.Java_Is_UnderControl.Vision.Cameras.Types.Interfaces.ICameraObject;
import frc.Java_Is_UnderControl.Vision.Cameras.Types.Interfaces.ICameraOdometry;
import frc.Java_Is_UnderControl.Vision.Cameras.Types.Limelight.LimelightHelpers.LimelightTarget_Retro;
import frc.Java_Is_UnderControl.Vision.Cameras.Types.Limelight.LimelightHelpers.PoseEstimate;
import frc.Java_Is_UnderControl.Vision.Object_Detection.ObjectDetection;
import frc.Java_Is_UnderControl.Vision.Object_Detection.ObjectDetectionCamera;
import frc.Java_Is_UnderControl.Vision.Object_Detection.ObjectPoseEstimationRobotOriented;
import frc.Java_Is_UnderControl.Vision.Odometry.PoseEstimation;

public class LimelightCamera implements ICameraOdometry, ICameraObject {

  private String cameraName;
  private String objectName;
  private double maxTx;
  private double maxTy;
  private Pose2d robotPose2D;
  private Rotation2d robotRotation;
  private Transform3d camToRobot;
  private boolean useMegaTag1;

  private CustomStringLogger stateOfPoseUpdate;

  private ObjectData objectData;
  private ObjectDetection objectDetection;
  private ObjectDetectionCamera objectDetectionCamera;
  private ObjectPoseEstimationRobotOriented objectPose;
  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

  private CustomDoubleLogger pitchLog;
  private CustomDoubleLogger yawLog;
  private CustomDoubleLogger areaLog;
  private CustomDoubleLogger distanceToAprilLog;
  private CustomPose3dLogger aprilPoseLog;
  private CustomTransform3dLogger camToRobotLog;
  private CustomIntegerLogger aprilIDLog;
  private CustomPose3dLogger robotPoseLog;

  private double pitch = 0;

  private double yaw = 0;

  private double area = 0;

  private double distanceToTarget = 0;

  private int aprilID = 0;

  private Pose3d robotPose = null;

  public LimelightCamera(String cameraName, Translation3d translation, Rotation3d rotation, boolean useMegaTag1) {
    this.useMegaTag1 = useMegaTag1;
    this.cameraName = cameraName;
    this.setCamToRobot(translation, rotation);
    this.stateOfPoseUpdate = new CustomStringLogger("/Vision/" + cameraName + "/StateOfPose");
    this.pitchLog = new CustomDoubleLogger(
        "/Vision/" + this.cameraName + "/pitch");
    this.yawLog = new CustomDoubleLogger(
        "/Vision/" + this.cameraName + "/yaw");
    this.areaLog = new CustomDoubleLogger(
        "/Vision/" + this.cameraName + "/area");
    this.distanceToAprilLog = new CustomDoubleLogger(
        "/Vision/" + this.cameraName + "/distanceToApril");
    this.aprilPoseLog = new CustomPose3dLogger(
        "/Vision/" + this.cameraName + "/aprilPose");
    this.camToRobotLog = new CustomTransform3dLogger("/Vision/" + this.cameraName
        + "/" + "cameraPosition");
    this.aprilIDLog = new CustomIntegerLogger("/Vision/" + this.cameraName + "/aprilID");
    this.robotPoseLog = new CustomPose3dLogger("/Vision" + this.cameraName + "/robotPose");
  }

  public LimelightCamera(String cameraName, Translation3d translation, Rotation3d rotation) {
    this(cameraName, translation, rotation, false);
  }

  public LimelightCamera(String cameraName, String objectName, double maxTx, double maxTy, Pose2d robotPose,
      Translation3d translation, Rotation3d rotation) {
    this.cameraName = cameraName;
    this.objectName = objectName;
    this.maxTx = maxTx;
    this.maxTy = maxTy;
    this.robotPose2D = robotPose;
    this.setCamToRobot(translation, rotation);
  }

  private void setCamToRobot(Translation3d translation, Rotation3d rotation) {
    this.camToRobot = new Transform3d(translation, rotation);
  }

  public String getCameraName() {
    return this.cameraName;
  }

  private double getDistanceAprilTag() {
    double ty = LimelightHelpers.getTY(cameraName);

    double limelightMountAngleDegrees = this.camToRobot.getRotation().getX();
    double limelightLensHeightInches = this.camToRobot.getZ();
    double goalHeightInches = this.aprilTagFieldLayout.getTagPose((int) LimelightHelpers.getFiducialID(cameraName))
        .get().getZ();
    double angleToGoalDegrees = limelightMountAngleDegrees + ty;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)
        / Math.tan(angleToGoalRadians);

    return distanceFromLimelightToGoalInches;
  }

  public void setAprilTagData() {
    if (LimelightHelpers.getTV(cameraName)) {
      this.setYaw(LimelightHelpers.getTX(cameraName));
      this.setPitch(LimelightHelpers.getTY(cameraName));
      this.setArea(LimelightHelpers.getTA(cameraName));
      this.setDistanceTarget(this.getDistanceAprilTag());
      this.setAprilID((int) LimelightHelpers.getFiducialID(cameraName));
      if (!this.getRobotPose().isEmpty()) {
        this.setRobotPose3d(this.getRobotPose().get().estimatedPose);
      }
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

  private void setAprilID(int aprilID) {
    this.aprilID = aprilID;
    this.updateLogsAprilTag();
  }

  private void setRobotPose3d(Pose3d robotPose) {
    this.robotPose = robotPose;
    this.updateLogsAprilTag();
  }

  @Override
  public Optional<PoseEstimation> getRobotPose() {
    PoseEstimate limelightPoseEstimate;
    if (LimelightHelpers.getTV(cameraName)) {
      if (useMegaTag1) {
        limelightPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(this.cameraName);
        this.stateOfPoseUpdate.append("GETTING_MEGATAG_1");
      } else {
        limelightPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(this.cameraName);
        this.stateOfPoseUpdate.append("GETTING_MEGATAG_2");
      }
      return Optional.of(this.convertPoseEstimate(limelightPoseEstimate));
    } else {
      return Optional.empty();
    }
  }

  private PoseEstimation convertPoseEstimate(PoseEstimate limelightPoseEstimate) {
    return new PoseEstimation(new Pose3d(limelightPoseEstimate.pose),
        limelightPoseEstimate.timestampSeconds,
        limelightPoseEstimate.tagCount, limelightPoseEstimate.avgTagDist);
  }

  @Override
  public void updateLogsAprilTag() {
    this.stateOfPoseUpdate.append(this.useMegaTag1 ? "MegaTag1" : "MegaTag2");
    if (LimelightHelpers.getTV(cameraName)) {
      this.pitchLog.append(this.pitch);
      this.yawLog.append(this.yaw);
      this.areaLog.append(this.area);
      this.camToRobotLog.appendDegrees(this.camToRobot);
      this.aprilIDLog.append(this.aprilID);
      this.distanceToAprilLog.append(this.distanceToTarget);
      if (robotPose != null) {
        this.robotPoseLog.appendDegrees(this.robotPose);
      }
    }
  }

  private double getDistanceObject(LimelightTarget_Retro target) {
    // Pose3d targetPose = target.getTargetPose_CameraSpace();
    // if (LimelightHelpers.getTV(this.cameraName)) {
    // double distanceToTarget =
    // PhotonUtils.calculateDistanceToTargetMeters(this.camToRobot.getY(),
    // this.getObjectPose().getY(),
    // Units.degreesToRadians(this.camToRobot.getX()),
    // Units.degreesToRadians(targetPose.getY()));
    // return distanceToTarget;
    // } else {
    return 0;
    // }
  }

  private void setObjectData() {
    // var results = LimelightHelpers.getLatestResults(cameraName);
    // if (results.targets_Retro.length > 0) {
    // var target = results.targets_Retro[0];
    // this.objectDetection = new ObjectDetection(
    // target.tx,
    // target.ty,
    // (this.objectName != null ? this.objectName : "object"));
    // this.objectData = new ObjectData(this.cameraName,
    // this.objectName,
    // objectDetection,
    // this.getDistanceObject(target),
    // this.camToRobot);
  }

  private void setObjectPose(double maxTy, double maxTx) {
    this.setObjectData();
    this.objectDetectionCamera = new ObjectDetectionCamera(this.camToRobot, maxTy, maxTx);
    this.objectPose = new ObjectPoseEstimationRobotOriented(this.objectDetection, this.objectDetectionCamera);
  }

  @Override
  public Pose2d getObjectPose() {
    this.setObjectPose(this.maxTy, this.maxTx);
    Pose2d objectPose2d = this.objectPose.getObjectPoseFromRobotCenter();
    return this.objectPose.getObjectPoseInField(this.robotPose2D, objectPose2d);
  }

  @Override
  public ObjectData getObjectData() {
    this.setObjectData();
    return this.objectData;
  }

  @Override
  public void updateLogsObject() {
    this.setObjectData();
    if (this.objectData != null) {
      this.objectData.updateLogs();
    }
  }

  public void setRobotRotation(Rotation2d rotation) {
    this.robotRotation = rotation;
  }

  public Rotation2d getRobotRotation() {
    return this.robotRotation;
  }

  public int getNumberOfTargetsDetected() {
    return LimelightHelpers.getTargetCount(cameraName);
  }
}
