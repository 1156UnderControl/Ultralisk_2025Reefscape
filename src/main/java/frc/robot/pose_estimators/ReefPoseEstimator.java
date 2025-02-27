package frc.robot.pose_estimators;

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
import frc.Java_Is_UnderControl.Vision.Odometry.PoseEstimation;
import frc.Java_Is_UnderControl.Vision.Odometry.PoseEstimator;
import frc.robot.constants.VisionConstants;

public class ReefPoseEstimator implements PoseEstimator {

  PhotonCamera arducamLeft;

  PhotonCamera arducamRight;

  PhotonPoseEstimator photonPoseEstimatorLeft;

  PhotonPoseEstimator photonPoseEstimatorRight;

  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  int numberOfTargetsUsedLeft = 0;

  CustomPose2dLogger detectedPoseLogger;

  CustomDoubleLogger numberOfDetectedTagsLogger;

  CustomDoubleLogger stdDevXYLogger;

  CustomDoubleLogger stdDevThetaLogger;

  CustomBooleanLogger isLeftDetectingLogger;

  CustomBooleanLogger isRightDetectingLogger;

  CustomStringLogger stateOfPoseUpdateLeft;

  CustomStringLogger stateOfPoseUpdateRight;

  private boolean arducamLeftNotSeeing = true;

  private boolean arducamRightNotSeeing = true;

  public ReefPoseEstimator(PhotonCamera arducamLeft, Transform3d cameraPositionLeft, PhotonCamera arducamRight,
      Transform3d cameraPositionRight) {
    this.arducamLeft = arducamLeft;
    this.arducamRight = arducamRight;
    this.aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    this.photonPoseEstimatorLeft = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
        cameraPositionLeft);
    this.photonPoseEstimatorRight = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
        cameraPositionRight);
    this.detectedPoseLogger = new CustomPose2dLogger("/Vision/ReefPoseEstimator/DetectedPose");
    this.numberOfDetectedTagsLogger = new CustomDoubleLogger("/Vision/MultiCameraEstimator/NumberOfDetectedTags");
    this.isLeftDetectingLogger = new CustomBooleanLogger(
        "/Vision/ReefPoseEstimator/" + arducamLeft.getName() + "/IsDetecting");
    this.isRightDetectingLogger = new CustomBooleanLogger(
        "/Vision/ReefPoseEstimator/" + arducamRight.getName() + "/IsDetecting");
    this.stateOfPoseUpdateLeft = new CustomStringLogger(
        "/Vision/ReefPoseEstimator/" + arducamLeft.getName() +
            "/StateOfPoseUpdate");
    this.stateOfPoseUpdateRight = new CustomStringLogger(
        "/Vision/ReefPoseEstimator/" + arducamRight.getName() +
            "/StateOfPoseUpdate");
    this.stdDevXYLogger = new CustomDoubleLogger("Vision/ReefPoseEstimator/stdDevXY");
    this.stdDevThetaLogger = new CustomDoubleLogger("Vision/ReefPoseEstimator/stdDevTheta");
  }

  public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose) {
    photonPoseEstimatorLeft.addHeadingData(Timer.getFPGATimestamp(), referencePose.getRotation());
    photonPoseEstimatorRight.addHeadingData(Timer.getFPGATimestamp(), referencePose.getRotation());
    List<PhotonPipelineResult> resultsLeft = arducamLeft.getAllUnreadResults();
    List<PhotonPipelineResult> resultsRight = arducamLeft.getAllUnreadResults();

    if (resultsLeft.isEmpty() && resultsRight.isEmpty()) {
      return Optional.empty();
    }

    PoseEstimation poseEstimationLeft = null;

    PoseEstimation poseEstimationRight = null;

    Optional<EstimatedRobotPose> photonPoseEstimationLeft = this.photonPoseEstimatorLeft
        .update(resultsLeft.get(resultsLeft.size() - 1));

    if (!photonPoseEstimationLeft.isPresent()) {
      arducamLeftNotSeeing = true;
      isLeftDetectingLogger.append(false);
      stateOfPoseUpdateLeft.append("NO_TARGETS");
    } else {
      poseEstimationLeft = convertPhotonPoseEstimation(photonPoseEstimationLeft.get());
      isLeftDetectingLogger.append(true);
    }

    Optional<EstimatedRobotPose> photonPoseEstimationRight = this.photonPoseEstimatorRight
        .update(resultsRight.get(resultsRight.size() - 1));

    if (!photonPoseEstimationRight.isPresent()) {
      arducamRightNotSeeing = true;
      isRightDetectingLogger.append(false);
      stateOfPoseUpdateRight.append("NO_TARGETS");
    } else {
      poseEstimationRight = convertPhotonPoseEstimation(photonPoseEstimationRight.get());
      isRightDetectingLogger.append(true);
    }

    if (poseEstimationLeft == null && poseEstimationRight == null) {
      return Optional.empty();
    }

    if (poseEstimationLeft != null && poseEstimationRight != null) {
      if (poseEstimationLeft.distanceToTag < poseEstimationRight.distanceToTag) {
        detectedPoseLogger.appendRadians(poseEstimationLeft.estimatedPose.toPose2d());
        return Optional.of(poseEstimationLeft);
      }
      detectedPoseLogger.appendRadians(poseEstimationRight.estimatedPose.toPose2d());
      return Optional.of(poseEstimationRight);
    } else if (poseEstimationLeft != null) {
      detectedPoseLogger.appendRadians(poseEstimationLeft.estimatedPose.toPose2d());
      return Optional.of(poseEstimationLeft);
    } else {
      detectedPoseLogger.appendRadians(poseEstimationRight.estimatedPose.toPose2d());
      return Optional.of(poseEstimationRight);
    }
  }

  private PoseEstimation convertPhotonPoseEstimation(EstimatedRobotPose photonPoseEstimation) {
    double avgTagDist = photonPoseEstimation.estimatedPose.getTranslation().getDistance(
        this.aprilTagFieldLayout.getTagPose(photonPoseEstimation.targetsUsed.get(photonPoseEstimation.targetsUsed.size()
            - 1).getFiducialId()).get().getTranslation());

    double stdDevXY = VisionConstants.xyStdDevCoefficient * Math.pow(avgTagDist, 2)
        / photonPoseEstimation.targetsUsed.size();
    this.stdDevXYLogger.append(stdDevXY);
    double stdDevTheta = Double.POSITIVE_INFINITY;
    this.stdDevThetaLogger.append(stdDevTheta);
    return new PoseEstimation(photonPoseEstimation.estimatedPose,
        photonPoseEstimation.timestampSeconds,
        photonPoseEstimation.targetsUsed.size(),
        avgTagDist,
        VecBuilder.fill(stdDevXY,
            stdDevXY, stdDevTheta));
  }

}
