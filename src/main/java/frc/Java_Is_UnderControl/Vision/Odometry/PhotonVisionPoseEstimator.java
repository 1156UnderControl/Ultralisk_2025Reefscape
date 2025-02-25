package frc.Java_Is_UnderControl.Vision.Odometry;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomBooleanLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomPose2dLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomStringLogger;

public class PhotonVisionPoseEstimator implements PoseEstimator {

  PhotonPoseEstimator photonPoseEstimator;

  PhotonCamera camera;

  boolean only2TagsMeasurements = false;

  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  CustomBooleanLogger isDetectingLogger;

  CustomPose2dLogger detectedPoseLogger;

  CustomDoubleLogger numberOfDetectedTagsLogger;

  CustomDoubleLogger distToTag;

  CustomStringLogger stateOfPoseUpdate;

  public PhotonVisionPoseEstimator(PhotonCamera camera, Transform3d cameraPosition) {
    this.camera = camera;
    this.aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    this.photonPoseEstimator = new PhotonPoseEstimator(this.aprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraPosition);
    this.photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    this.detectedPoseLogger = new CustomPose2dLogger(
        "/Vision/PhotonVisionPoseEstimator/" + camera.getName() + "/DetectedPose");
    this.numberOfDetectedTagsLogger = new CustomDoubleLogger(
        "/Vision/PhotonVisionPoseEstimator/" + camera.getName() +
            "/NumberOfDetectedTags");
    this.distToTag = new CustomDoubleLogger(
        "/Vision/PhotonVisionPoseEstimator/" + camera.getName() + "/DistanceToTag");
    this.isDetectingLogger = new CustomBooleanLogger(
        "/Vision/PhotonVisionPoseEstimator/" + camera.getName() + "/IsDetecting");
    this.stateOfPoseUpdate = new CustomStringLogger(
        "/Vision/PhotonVisionPoseEstimator/" + camera.getName() +
            "/StateOfPoseUpdate");
  }

  public PhotonVisionPoseEstimator(PhotonCamera camera, Transform3d cameraPosition, boolean only2TagsMeasurements) {
    this.only2TagsMeasurements = only2TagsMeasurements;
    this.camera = camera;
    this.aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    this.photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        cameraPosition);
    this.photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
    this.detectedPoseLogger = new CustomPose2dLogger(
        "/Vision/PhotonVisionPoseEstimator/" + camera.getName() + "/DetectedPose");
    this.numberOfDetectedTagsLogger = new CustomDoubleLogger(
        "/Vision/PhotonVisionPoseEstimator/" + camera.getName() +
            "/NumberOfDetectedTags");
    this.distToTag = new CustomDoubleLogger(
        "/Vision/PhotonVisionPoseEstimator/" + camera.getName() + "/DistanceToTag");
    this.isDetectingLogger = new CustomBooleanLogger(
        "/Vision/PhotonVisionPoseEstimator/" + camera.getName() + "/IsDetecting");
    this.stateOfPoseUpdate = new CustomStringLogger(
        "/Vision/PhotonVisionPoseEstimator/" + camera.getName() +
            "/StateOfPoseUpdate");
  }

  public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose) {
    this.photonPoseEstimator.setReferencePose(referencePose);
    var results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      return Optional.empty();
    }
    Optional<EstimatedRobotPose> photonPoseEstimation = this.photonPoseEstimator
        .update(results.get(results.size() - 1));
    if (!photonPoseEstimation.isPresent()) {
      isDetectingLogger.append(false);
      stateOfPoseUpdate.append("NO_TARGETS");
      return Optional.empty();
    }
    isDetectingLogger.append(true);
    PoseEstimation poseEstimation = convertPhotonPoseEstimation(photonPoseEstimation.get());
    if (only2TagsMeasurements && poseEstimation.numberOfTargetsUsed < 2) {
      stateOfPoseUpdate.append("LESS_THAN_2_TARGETS");
      return Optional.empty();
    }
    if (poseEstimation.numberOfTargetsUsed == 1) {
      if (photonPoseEstimation.get().targetsUsed.get(0).getPoseAmbiguity() > 0.35) {
        stateOfPoseUpdate.append("HIGH_AMBIGUITY");
        return Optional.empty();
      }

      if (poseEstimation.distanceToTag > 2.7) {
        stateOfPoseUpdate.append("TOO_FAR_FROM_ONE_TAG");
        return Optional.empty();
      }
    }
    distToTag.append(poseEstimation.distanceToTag);
    numberOfDetectedTagsLogger.append(poseEstimation.numberOfTargetsUsed);
    detectedPoseLogger.appendRadians(poseEstimation.estimatedPose.toPose2d());
    stateOfPoseUpdate.append("UPDATING_POSE");
    return Optional.of(poseEstimation);
  }

  private PoseEstimation convertPhotonPoseEstimation(EstimatedRobotPose photonPoseEstimation) {

    return new PoseEstimation(photonPoseEstimation.estimatedPose,
        photonPoseEstimation.timestampSeconds,
        photonPoseEstimation.targetsUsed.size(),
        photonPoseEstimation.targetsUsed.get(photonPoseEstimation.targetsUsed.size()
            - 1).getBestCameraToTarget()
            .getX());
  }

}
