package frc.Java_Is_UnderControl.Vision.Odometry;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomBooleanLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomPose2dLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomStringLogger;
import frc.robot.constants.VisionConstants;

public class PhotonVisionPoseEstimator implements PoseEstimator {

  PhotonPoseEstimator photonPoseEstimator;

  PhotonCamera camera;

  boolean only2TagsMeasurements = false;

  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  private CustomBooleanLogger isDetectingLogger;

  private CustomPose2dLogger detectedPoseLogger;

  private CustomDoubleLogger numberOfDetectedTagsLogger;

  private CustomDoubleLogger distToTag;

  CustomStringLogger stateOfPoseUpdate;

  CustomDoubleLogger stdDevXYLogger;

  private CustomDoubleLogger stdDevThetaLogger;

  private Boolean useVisionHeadingCorrection = false;

  public PhotonVisionPoseEstimator(PhotonCamera camera, Transform3d cameraPosition) {
    this.camera = camera;
    this.aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    this.photonPoseEstimator = new PhotonPoseEstimator(this.aprilTagFieldLayout,
        PoseStrategy.PNP_DISTANCE_TRIG_SOLVE, cameraPosition);
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
    this.stdDevXYLogger = new CustomDoubleLogger("Vision/PhotonVisionPoseEstimator/" + camera.getName() + "/stdDevXY");
    this.stdDevThetaLogger = new CustomDoubleLogger(
        "Vision/PhotonVisionPoseEstimator/" + camera.getName() + "/stdDevTheta");
  }

  public PhotonVisionPoseEstimator(PhotonCamera camera, Transform3d cameraPosition, boolean only2TagsMeasurements) {
    this.only2TagsMeasurements = only2TagsMeasurements;
    this.camera = camera;
    this.aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    this.photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CONSTRAINED_SOLVEPNP,
        cameraPosition);
    // this.photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
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
    this.stdDevXYLogger = new CustomDoubleLogger("Vision/PhotonVisionPoseEstimator/" + camera.getName() + "/stdDevXY");
    this.stdDevThetaLogger = new CustomDoubleLogger(
        "Vision/PhotonVisionPoseEstimator/" + camera.getName() + "/stdDevTheta");
  }

  public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose) {
    try {
      this.photonPoseEstimator.setReferencePose(referencePose);
      this.photonPoseEstimator.addHeadingData(Timer.getFPGATimestamp(), referencePose.getRotation());
      List<PhotonPipelineResult> results = camera.getAllUnreadResults();
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
    } catch (Exception e) {
      return Optional.empty();
    }
  }

  private PoseEstimation convertPhotonPoseEstimation(EstimatedRobotPose photonPoseEstimation) {
    double avgTagDist = photonPoseEstimation.estimatedPose.getTranslation().getDistance(
        this.aprilTagFieldLayout.getTagPose(photonPoseEstimation.targetsUsed.get(photonPoseEstimation.targetsUsed.size()
            - 1).getFiducialId()).get().getTranslation());

    double stdDevXY = VisionConstants.xyStdDevCoefficient * Math.pow(avgTagDist, 2)
        / photonPoseEstimation.targetsUsed.size();
    this.stdDevXYLogger.append(stdDevXY);
    double stdDevTheta = useVisionHeadingCorrection ? VisionConstants.thetaStdDevCoefficient * Math.pow(avgTagDist, 2)
        / photonPoseEstimation.targetsUsed.size() : Double.POSITIVE_INFINITY;
    this.stdDevThetaLogger.append(stdDevTheta);
    return new PoseEstimation(photonPoseEstimation.estimatedPose,
        photonPoseEstimation.timestampSeconds,
        photonPoseEstimation.targetsUsed.size(),
        avgTagDist,
        VecBuilder.fill(stdDevXY,
            stdDevXY, stdDevTheta));
  }

  public String getEstimatorName() {
    return this.camera.getName();
  }

}
