package frc.robot.pose_estimators;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomBooleanLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomPose2dLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomStringLogger;
import frc.Java_Is_UnderControl.Vision.Odometry.PoseEstimation;
import frc.Java_Is_UnderControl.Vision.Odometry.PoseEstimator;
import frc.robot.constants.SwerveConstants.TargetBranch;
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

  CustomBooleanLogger isArightTargetLogger;

  CustomStringLogger stateOfPoseUpdateLeft;

  CustomStringLogger stateOfPoseUpdateRight;

  private Supplier<TargetBranch> targetBranchSupplier;

  private int[] desiredApriltagsIDs = new int[2];

  public ReefPoseEstimator(PhotonCamera arducamLeft, Transform3d cameraPositionLeft, PhotonCamera arducamRight,
      Transform3d cameraPositionRight, Supplier<TargetBranch> targetBranchSupplier) {
    this.arducamLeft = arducamLeft;
    this.arducamRight = arducamRight;
    this.aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    this.photonPoseEstimatorLeft = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CONSTRAINED_SOLVEPNP,
        cameraPositionLeft);
    this.photonPoseEstimatorRight = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CONSTRAINED_SOLVEPNP,
        cameraPositionRight);
    this.detectedPoseLogger = new CustomPose2dLogger("/Vision/ReefPoseEstimator/DetectedPose");
    this.numberOfDetectedTagsLogger = new CustomDoubleLogger("/Vision/ReefPoseEstimator/NumberOfDetectedTags");
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
    this.isArightTargetLogger = new CustomBooleanLogger(
        "/Vision/ReefPoseEstimator/IsARightTarget");
    this.targetBranchSupplier = targetBranchSupplier;
  }

  public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose) {
    boolean isArightBranchTarget;

    switch (targetBranchSupplier.get()) {
      case A:
        isArightBranchTarget = false;
        this.desiredApriltagsIDs = new int[] { 18, 7 };
        break;
      case B:
        isArightBranchTarget = true;
        this.desiredApriltagsIDs = new int[] { 18, 7 };
        break;
      case C:
        isArightBranchTarget = false;
        this.desiredApriltagsIDs = new int[] { 17, 7 };
        break;
      case D:
        isArightBranchTarget = true;
        this.desiredApriltagsIDs = new int[] { 17, 7 };
        break;
      case E:
        isArightBranchTarget = false;
        this.desiredApriltagsIDs = new int[] { 22, 9 };
        break;
      case F:
        isArightBranchTarget = true;
        this.desiredApriltagsIDs = new int[] { 22, 9 };
        break;
      case G:
        isArightBranchTarget = false;
        this.desiredApriltagsIDs = new int[] { 21, 10 };
        break;
      case H:
        isArightBranchTarget = true;
        this.desiredApriltagsIDs = new int[] { 21, 10 };
        break;
      case I:
        isArightBranchTarget = false;
        this.desiredApriltagsIDs = new int[] { 20, 11 };
        break;
      case J:
        isArightBranchTarget = true;
        this.desiredApriltagsIDs = new int[] { 20, 11 };
        break;
      case K:
        isArightBranchTarget = false;
        this.desiredApriltagsIDs = new int[] { 19, 6 };
        break;
      case L:
        isArightBranchTarget = true;
        this.desiredApriltagsIDs = new int[] { 19, 6 };
        break;
      default:
        isArightBranchTarget = false;
    }

    isArightTargetLogger.append(isArightBranchTarget);

    List<PhotonPipelineResult> resultsLeft = arducamLeft.getAllUnreadResults();
    List<PhotonPipelineResult> resultsRight = arducamRight.getAllUnreadResults();

    if (resultsLeft.isEmpty() && resultsRight.isEmpty()) {
      return Optional.empty();
    }

    PoseEstimation poseEstimationLeft = null;

    PoseEstimation poseEstimationRight = null;

    if (!resultsLeft.isEmpty() && isArightBranchTarget) {
      Optional<EstimatedRobotPose> photonPoseEstimationLeft = this.photonPoseEstimatorLeft
          .update(resultsLeft.get(resultsLeft.size() - 1));

      if (!photonPoseEstimationLeft.isPresent()) {
        isLeftDetectingLogger.append(false);
        stateOfPoseUpdateLeft.append("NO_TARGETS");
      } else {
        if (photonPoseEstimationLeft.get().targetsUsed.stream().anyMatch(
            t -> t.fiducialId == this.desiredApriltagsIDs[0] || t.fiducialId == this.desiredApriltagsIDs[1])) {
          poseEstimationLeft = convertPhotonPoseEstimation(photonPoseEstimationLeft.get());
          isLeftDetectingLogger.append(true);
        } else {
          isLeftDetectingLogger.append(true);
          stateOfPoseUpdateLeft.append("WITHOUT_DESIRED_APRILTAGS" + Arrays.toString(this.desiredApriltagsIDs));
        }
      }
    } else {
      stateOfPoseUpdateLeft.append("NO_TARGETS_OR_TARGET_BRANCH_IS_LEFT");
    }

    if (!resultsRight.isEmpty() && !isArightBranchTarget) {
      Optional<EstimatedRobotPose> photonPoseEstimationRight = this.photonPoseEstimatorRight
          .update(resultsRight.get(resultsRight.size() - 1));

      if (!photonPoseEstimationRight.isPresent()) {
        isRightDetectingLogger.append(false);
        stateOfPoseUpdateRight.append("NO_TARGETS");
      } else {
        if (photonPoseEstimationRight.get().targetsUsed.stream().anyMatch(
            t -> t.fiducialId == this.desiredApriltagsIDs[0] || t.fiducialId == this.desiredApriltagsIDs[1])) {
          poseEstimationRight = convertPhotonPoseEstimation(photonPoseEstimationRight.get());
          isRightDetectingLogger.append(true);
        } else {
          isRightDetectingLogger.append(true);
          stateOfPoseUpdateRight.append("WITHOUT_DESIRED_APRILTAGS" + Arrays.toString(this.desiredApriltagsIDs));
        }
      }
    } else {
      stateOfPoseUpdateRight.append("NO_TARGETS_OR_TARGET_BRANCH_IS_RIGHT");
    }

    if (poseEstimationLeft == null && poseEstimationRight == null) {
      return Optional.empty();
    }

    if (poseEstimationLeft != null && poseEstimationRight != null) {
      if (poseEstimationLeft.distanceToTag < poseEstimationRight.distanceToTag) {
        return returnPoseEstimationLeftCamera(poseEstimationLeft);
      }
      return returnPoseEstimationRightCamera(poseEstimationRight);
    } else if (poseEstimationLeft != null) {
      return returnPoseEstimationLeftCamera(poseEstimationLeft);
    } else {
      return returnPoseEstimationRightCamera(poseEstimationRight);
    }
  }

  private Optional<PoseEstimation> returnPoseEstimationRightCamera(PoseEstimation poseEstimationRight) {
    stateOfPoseUpdateRight.append("UPDATING_WITH_RIGHT_CAMERA");
    detectedPoseLogger.appendRadians(poseEstimationRight.estimatedPose.toPose2d());
    return Optional.of(poseEstimationRight);
  }

  private Optional<PoseEstimation> returnPoseEstimationLeftCamera(PoseEstimation poseEstimationLeft) {
    stateOfPoseUpdateLeft.append("UPDATING_WITH_LEFT_CAMERA");
    detectedPoseLogger.appendRadians(poseEstimationLeft.estimatedPose.toPose2d());
    return Optional.of(poseEstimationLeft);
  }

  public void setHeadingMeasurement(Rotation2d heading) {
    photonPoseEstimatorLeft.addHeadingData(Timer.getFPGATimestamp(), heading);
    photonPoseEstimatorRight.addHeadingData(Timer.getFPGATimestamp(), heading);
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
