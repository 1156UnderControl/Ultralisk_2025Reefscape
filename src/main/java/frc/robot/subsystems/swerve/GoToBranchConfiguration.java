package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import frc.Java_Is_UnderControl.Util.CoordinatesTransform;
import frc.Java_Is_UnderControl.Util.GeomUtil;
import frc.robot.constants.FieldConstants.Algae.AlgaeHeightReef;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.constants.SwerveConstants.AutoAlignConstants;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.constants.SwerveConstants.TargetFace;

public class GoToBranchConfiguration {
  private TargetBranch branch;
  private TargetFace face;
  private double minErrorPose;
  private double maxErrorPose;
  private double errorForRotationAlignPose;
  private double maxVelocity;
  private double minVelocity;
  private String goToBranchMode;
  private boolean goDirect;

  private double goToPoseTranslationDeadband = AutoAlignConstants.PoseDeadBand.POSE_TRANSLATION_DEADBAND_BACKUP;;
  private double goToPoseHeadingDeadband = AutoAlignConstants.PoseDeadBand.POSE_HEADING_DEADBAND_BACKUP;

  private Pose2d finalTargetPose;

  private double finalVelocity;

  private boolean canDriveAimingAtPose = false;

  private String state;

  double distanceToTargetBranch;
  double distanceToTargetFace;
  double distanceToTarget;
  double deltaVelocity;
  double deltaDistance;
  double relationDeltas;
  double targetVelocity;

  public GoToBranchConfiguration(double minErrorPose,
      double maxErrorPose, double errorForRotationAlignPose, String goToBranchMode,
      double minVelocity, double maxVelocity) {
    this.minErrorPose = minErrorPose;
    this.maxErrorPose = maxErrorPose;
    this.errorForRotationAlignPose = errorForRotationAlignPose;
    this.goToBranchMode = goToBranchMode;
    this.minVelocity = minVelocity;
    this.maxVelocity = maxVelocity;
  }

  public void updateBranchData(Pose2d robotPose, Supplier<ReefLevel> scorerTargetReefLevelSupplier,
      Supplier<AlgaeHeightReef> scorerTargetReefLevelAlgaeSupplier,
      Supplier<Boolean> elevatorAtHighPositionSupplier, boolean backup, boolean goToFace) {
    if (goToFace) {
      this.face = this.getReefFace(branch);
      this.distanceToTargetFace = face.getTargetPoseToScore().getTranslation().getDistance(robotPose.getTranslation());
      Pose2d targetFaceScorePose;
      if (scorerTargetReefLevelAlgaeSupplier.get() == AlgaeHeightReef.LOW
          || scorerTargetReefLevelAlgaeSupplier.get() == AlgaeHeightReef.MID) {
        targetFaceScorePose = backup ? CoordinatesTransform.getRetreatPose(face.getTargetPoseToScore(), 0.05)
            : CoordinatesTransform.getRetreatPose(face.getTargetPoseToScore(), 0.30);
      } else {
        targetFaceScorePose = face.getTargetPoseToScore();
      }
      distanceToTarget = this.getDriveTarget(robotPose, targetFaceScorePose, true, false).getTranslation()
          .getDistance(robotPose.getTranslation());
      finalVelocity = this.calculateRobotMaxVelocity(distanceToTargetFace, this.maxVelocity, this.minVelocity,
          this.maxErrorPose, minErrorPose);
      finalTargetPose = this.getDriveTarget(robotPose, targetFaceScorePose, this.goDirect, false);
      this.canDriveAimingAtPose = false;
      this.state = "DRIVE_TO_FACE_" + this.goToBranchMode + branch.name() + "COLLECT:" + !backup;
    } else {
      this.distanceToTargetBranch = branch.getTargetPoseToScore().getTranslation()
          .getDistance(robotPose.getTranslation());
      Pose2d targetBranchScorePose = scorerTargetReefLevelSupplier.get() == ReefLevel.L4
          ? CoordinatesTransform.getRetreatPose(branch.getTargetPoseToScore(), 0.09)
          : branch.getTargetPoseToScore();
      this.distanceToTarget = this.getDriveTarget(robotPose, targetBranchScorePose, this.goDirect, backup)
          .getTranslation().getDistance(robotPose.getTranslation());
      finalVelocity = this.calculateRobotMaxVelocity(distanceToTargetBranch, this.maxVelocity, this.minVelocity,
          this.maxErrorPose, minErrorPose);
      finalTargetPose = this.getDriveTarget(robotPose, targetBranchScorePose, this.goDirect, backup);
      if (distanceToTargetBranch <= this.errorForRotationAlignPose) {
        this.canDriveAimingAtPose = true;
      } else {
        this.canDriveAimingAtPose = false;
      }
      if (distanceToTargetBranch <= minErrorPose) {
        this.state = "DRIVE_TO_BRANCH_" + this.goToBranchMode + branch.name() + "_CLOSE";
        return;
      } else if (distanceToTargetBranch <= maxErrorPose) {
        this.state = "DRIVE_TO_BRANCH_" + this.goToBranchMode + branch.name() + "_FAR";
        return;
      } else {
        this.state = "DRIVE_TO_BRANCH_" + this.goToBranchMode + branch.name() + "_MID";
        return;
      }
    }
  }

  private Pose2d getDriveTarget(Pose2d robot, Pose2d goal, boolean goDirect, boolean backup) {
    if (backup) {
      goal = goal.transformBy(GeomUtil.toTransform2d(-0.35, 0.0));
      this.state = "DRIVE_TO_BRANCH_" + this.goToBranchMode + branch.name() + "BACKUP";
    } else {
      goal = goal.transformBy(GeomUtil.toTransform2d(-0.11, 0.0));
    }
    if (goDirect) {
      return goal;
    } else {
      this.goToPoseTranslationDeadband = AutoAlignConstants.PoseDeadBand.POSE_TRANSLATION_DEADBAND;
      this.goToPoseHeadingDeadband = AutoAlignConstants.PoseDeadBand.POSE_HEADING_DEADBAND;
      return this.calculateReefAvoidenceTarget(robot, goal);
    }
  }

  private Pose2d calculateReefAvoidenceTarget(Pose2d robot, Pose2d goal) {
    var offset = robot.relativeTo(goal);
    double yDistance = Math.abs(offset.getY());
    double xDistance = Math.abs(offset.getX());
    double shiftXT = MathUtil.clamp(
        (yDistance / (Reef.faceLength * 2)) + ((xDistance - 0.3) / (Reef.faceLength * 3)),
        0.0,
        1.0);
    double shiftYT = MathUtil.clamp(offset.getX() / Reef.faceLength, 0.0, 1.0);
    return goal.transformBy(
        GeomUtil.toTransform2d(
            -shiftXT * 1.2,
            Math.copySign(shiftYT * 1.5 * 0.8, offset.getY())));
  }

  private double calculateRobotMaxVelocity(double distanceToTarget, double maxVelocity, double minVelocity,
      double maxDistance, double minDistance) {
    this.deltaVelocity = maxVelocity - minVelocity;
    this.deltaDistance = maxDistance - minDistance;
    this.relationDeltas = deltaVelocity / deltaDistance;
    this.targetVelocity = minVelocity + relationDeltas * (distanceToTarget - minDistance);
    if (distanceToTarget > maxDistance) {
      return maxVelocity;
    }
    if (distanceToTarget < minDistance) {
      return minVelocity;
    }
    return targetVelocity;
  }

  private TargetFace getReefFace(TargetBranch targetBranch) {
    TargetFace targetFaceAssignment;

    switch (targetBranch) {
      case A:
        targetFaceAssignment = TargetFace.A;
        break;
      case B:
        targetFaceAssignment = TargetFace.A;
        break;
      case C:
        targetFaceAssignment = TargetFace.B;
        break;
      case D:
        targetFaceAssignment = TargetFace.B;
        break;
      case E:
        targetFaceAssignment = TargetFace.C;
        break;
      case F:
        targetFaceAssignment = TargetFace.C;
        break;
      case G:
        targetFaceAssignment = TargetFace.D;
        break;
      case H:
        targetFaceAssignment = TargetFace.D;
        break;
      case I:
        targetFaceAssignment = TargetFace.E;
        break;
      case J:
        targetFaceAssignment = TargetFace.E;
        break;
      case K:
        targetFaceAssignment = TargetFace.F;
        break;
      case L:
        targetFaceAssignment = TargetFace.F;
        break;
      default:
        targetFaceAssignment = TargetFace.A;
        break;
    }
    return targetFaceAssignment;
  }

  public double getPoseTranslationDeadband() {
    return this.goToPoseTranslationDeadband;
  }

  public double getPoseHeadingDeadband() {
    return this.goToPoseHeadingDeadband;
  }

  public String getGoToState() {
    return this.state;
  }

  public Pose2d getFinalPose() {
    return this.finalTargetPose;
  }

  public double getFinalVelocity() {
    return this.finalVelocity;
  }

  public boolean canDriveAimingAtPose() {
    return this.canDriveAimingAtPose;
  }

  public void setBranch(TargetBranch targetBranch, boolean goDirect) {
    this.branch = targetBranch;
    this.goDirect = goDirect;
  }

  public void setFace(TargetFace targetFace) {
    this.face = targetFace;
  }

  public double getDistanceToTargetBranch() {
    return this.distanceToTargetBranch;
  }

  public double getDistanceToTargetFace() {
    return this.distanceToTargetFace;
  }

  public TargetFace getTargetFace() {
    return getReefFace(branch);
  }
}
