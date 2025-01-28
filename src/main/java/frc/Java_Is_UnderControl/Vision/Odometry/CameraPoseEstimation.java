package frc.Java_Is_UnderControl.Vision.Odometry;

import java.util.Optional;

import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomBooleanLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomPose2dLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomStringLogger;
import frc.Java_Is_UnderControl.Vision.Cameras.Data.AprilTagData;
import frc.Java_Is_UnderControl.Vision.Cameras.Types.Interfaces.ICameraOdometry;

public class CameraPoseEstimation implements PoseEstimator {

  ICameraOdometry camera;
  AprilTagData aprilTagData;
  Transform3d cameraPosition;

  boolean only2TagsMeasurements = false;

  CustomBooleanLogger isDetectingLogger;

  CustomPose2dLogger detectedPoseLogger;

  CustomDoubleLogger numberOfDetectedTagsLogger;

  CustomStringLogger stateOfPoseUpdate;

  PhotonPoseEstimator photonPoseEstimator;

  public CameraPoseEstimation(ICameraOdometry camera, AprilTagData aprilTagData, Transform3d cameraPosition) {
    this(camera, aprilTagData, cameraPosition, false);
  }

  public CameraPoseEstimation(ICameraOdometry camera, AprilTagData aprilTagData, Transform3d cameraPosition,
      boolean only2TagsMeasurements) {
    this.only2TagsMeasurements = only2TagsMeasurements;
    this.camera = camera;
    this.aprilTagData = aprilTagData;
    this.cameraPosition = cameraPosition;

    this.detectedPoseLogger = new CustomPose2dLogger(
        "/Vision/PhotonVisionPoseEstimator/" + aprilTagData.getCameraName() + "/DetectedPose");
    this.numberOfDetectedTagsLogger = new CustomDoubleLogger(
        "/Vision/PhotonVisionPoseEstimator/" + aprilTagData.getCameraName() + "/NumberOfDetectedTags");
    this.isDetectingLogger = new CustomBooleanLogger(
        "/Vision/PhotonVisionPoseEstimator/" + aprilTagData.getCameraName() + "/IsDetecting");
    this.stateOfPoseUpdate = new CustomStringLogger(
        "/Vision/PhotonVisionPoseEstimator/" + aprilTagData.getCameraName() + "/StateOfPoseUpdate");
  }

  @Override
  public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose) {
    if (!aprilTagData.hasTargets()) {
      isDetectingLogger.append(false);
      stateOfPoseUpdate.append("NO_TARGETS");
      return Optional.empty();
    }
    isDetectingLogger.append(true);
    PoseEstimation poseEstimation = camera.getRobotPose().get();
    if (only2TagsMeasurements && poseEstimation.numberOfTargetsUsed < 2) {
      stateOfPoseUpdate.append("LESS_THAN_2_TARGETS");
      return Optional.empty();
    }
    if (poseEstimation.distanceToTag > 2.7) {
      stateOfPoseUpdate.append("TOO_FAR_FROM_ONE_TAG");
      return Optional.empty();
    }
    this.numberOfDetectedTagsLogger.append(poseEstimation.numberOfTargetsUsed);
    this.detectedPoseLogger.appendRadians(poseEstimation.estimatedPose.toPose2d());
    this.stateOfPoseUpdate.append("UPDATING_POSE");
    return Optional.of(poseEstimation);
  }
}
