package frc.Java_Is_UnderControl.Vision.Cameras.Types.Limelight;

import java.util.Optional;

import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomStringLogger;
import frc.Java_Is_UnderControl.Vision.Cameras.Data.AprilTagData;
import frc.Java_Is_UnderControl.Vision.Cameras.Data.ObjectData;
import frc.Java_Is_UnderControl.Vision.Cameras.Types.Interfaces.ICameraObject;
import frc.Java_Is_UnderControl.Vision.Cameras.Types.Interfaces.ICameraOdometry;
import frc.Java_Is_UnderControl.Vision.Cameras.Types.Limelight.LimelightHelpers.LimelightTarget_Fiducial;
import frc.Java_Is_UnderControl.Vision.Cameras.Types.Limelight.LimelightHelpers.LimelightTarget_Retro;
import frc.Java_Is_UnderControl.Vision.Object_Detection.ObjectDetection;
import frc.Java_Is_UnderControl.Vision.Object_Detection.ObjectDetectionCamera;
import frc.Java_Is_UnderControl.Vision.Object_Detection.ObjectPoseEstimationRobotOriented;
import frc.Java_Is_UnderControl.Vision.Odometry.PoseEstimation;

public class LimelightCamera implements ICameraOdometry, ICameraObject {

  private String cameraName;
  private String objectName;
  private double maxTx;
  private double maxTy;
  private Pose2d robotPose;
  private Rotation2d robotRotation;
  private Transform3d camToRobot;
  private boolean useMegaTag1;

  private CustomStringLogger stateOfPoseUpdate;

  private AprilTagData aprilData;
  private ObjectData objectData;
  private ObjectDetection objectDetection;
  private ObjectDetectionCamera objectDetectionCamera;
  private ObjectPoseEstimationRobotOriented objectPose;
  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

  public LimelightCamera(String cameraName, Translation3d translation, Rotation3d rotation) {
    this.cameraName = cameraName;
    this.setCamToRobot(translation, rotation);
  }

  public LimelightCamera(String cameraName, Translation3d translation, Rotation3d rotation, boolean useMegaTag1) {
    this.useMegaTag1 = useMegaTag1;
    this.cameraName = cameraName;
    this.setCamToRobot(translation, rotation);
    this.stateOfPoseUpdate = new CustomStringLogger("/Vision/" + cameraName + "/StateOfPose");
  }

  public LimelightCamera(String cameraName, String objectName, double maxTx, double maxTy, Pose2d robotPose,
      Translation3d translation, Rotation3d rotation) {
    this.cameraName = cameraName;
    this.objectName = objectName;
    this.maxTx = maxTx;
    this.maxTy = maxTy;
    this.robotPose = robotPose;
    this.setCamToRobot(translation, rotation);
  }

  private void setCamToRobot(Translation3d translation, Rotation3d rotation) {
    this.camToRobot = new Transform3d(translation, rotation);
  }

  public String getCameraName() {
    return this.cameraName;
  }

  private double getDistanceAprilTag(LimelightTarget_Fiducial target, int targetID) {
    Pose3d targetPose = target.getTargetPose_CameraSpace();
    if (LimelightHelpers.getTV(this.cameraName)) {
      double distanceToTarget = PhotonUtils.calculateDistanceToTargetMeters(this.camToRobot.getY(),
          aprilTagFieldLayout.getTagPose(targetID).get().getZ(),
          Units.degreesToRadians(this.camToRobot.getX()), Units.degreesToRadians(targetPose.getY()));
      return distanceToTarget;
    } else {
      return 0;
    }
  }

  private Optional<AprilTagData> setAprilTagData() {
    var result = LimelightHelpers.getLatestResults(cameraName);
    double[] aprilIds = new double[this.aprilTagFieldLayout.getTags().size()];
    if (LimelightHelpers.getTV(this.cameraName)) {
      for (LimelightTarget_Fiducial target : result.targets_Fiducials) {
        AprilTagData aprilTagData;

        if (Double.isNaN(aprilIds[(int) target.fiducialID])
            && (((aprilIds[(int) target.fiducialID] - (target.ty + target.tx + target.ta)) > 0.001)
                || (aprilIds[(int) target.fiducialID]
                    - (target.ty + target.tx + target.ta)) < -0.001)) {
          aprilTagData = new AprilTagData(cameraName, (int) target.fiducialID, target.tx, target.ty,
              target.ta, this.getDistanceAprilTag(target, (int) target.fiducialID), this.camToRobot,
              this.getNumberOfTargetsDetected());
          aprilIds[aprilTagData.getAprilID()] = aprilTagData.getPitch() + aprilTagData.getYaw()
              + aprilTagData.getArea();
          return Optional.of(aprilTagData);
        } else {
          return Optional.empty();
        }
      }
      return Optional.empty();
    } else {
      return Optional.empty();
    }
  }

  @Override
  public Optional<PoseEstimation> getRobotPose() {
    if (LimelightHelpers.getTV(this.cameraName)) {
      LimelightHelpers.PoseEstimate estimate;
      if (useMegaTag1) {
        estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
      } else {
        estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
      }
      return Optional.of(new PoseEstimation(new Pose3d(estimate.pose),
          estimate.timestampSeconds,
          estimate.tagCount,
          estimate.avgTagDist));
    } else {
      return Optional.empty();
    }
  }

  @Override
  public Optional<AprilTagData> getAprilTagData() {
    return this.setAprilTagData();
  }

  @Override
  public void updateLogsAprilTag() {
    this.setAprilTagData();
    if (this.aprilData != null) {
      this.stateOfPoseUpdate.append(this.useMegaTag1 ? "MegaTag1" : "MegaTag2");
      this.aprilData.updateLogs();
    }
  }

  private double getDistanceObject(LimelightTarget_Retro target) {
    Pose3d targetPose = target.getTargetPose_CameraSpace();
    if (LimelightHelpers.getTV(this.cameraName)) {
      double distanceToTarget = PhotonUtils.calculateDistanceToTargetMeters(this.camToRobot.getY(),
          this.getObjectPose().getY(),
          Units.degreesToRadians(this.camToRobot.getX()), Units.degreesToRadians(targetPose.getY()));
      return distanceToTarget;
    } else {
      return 0;
    }
  }

  private void setObjectData() {
    var results = LimelightHelpers.getLatestResults(cameraName);
    if (results.targets_Retro.length > 0) {
      var target = results.targets_Retro[0];
      this.objectDetection = new ObjectDetection(
          target.tx,
          target.ty,
          (this.objectName != null ? this.objectName : "object"));
      this.objectData = new ObjectData(this.cameraName,
          this.objectName,
          objectDetection,
          this.getDistanceObject(target),
          this.camToRobot);
    }
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
    return this.objectPose.getObjectPoseInField(this.robotPose, objectPose2d);
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
