package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import frc.Java_Is_UnderControl.Util.CoordinatesTransform;
import frc.Java_Is_UnderControl.Util.GeomUtil;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.constants.SwerveConstants.AutoAlignConstants;
import frc.robot.constants.SwerveConstants.TargetBranch;

public class GoToBranchConfiguration {
  private TargetBranch branch;
  private boolean backupBranch;
  private double minErrorPose;
  private double midErrorPose;
  private double maxErrorPose;
  private double errorForRotationAlignPose;
  private double errorElevatorRaisedPose;
  private double maxVelocity;
  private double midVelocity;
  private double minVelocity;
  private double ultraVelocity;
  private String goToBranchMode;
  private boolean useMidError;
  private boolean useRotationAlignPose;
  private boolean useErrorElevatorRaisedPose;

  private double goToPoseTranslationDeadband = AutoAlignConstants.PoseDeadBand.POSE_TRANSLATION_DEADBAND_BACKUP;;
  private double goToPoseHeadingDeadband = AutoAlignConstants.PoseDeadBand.POSE_HEADING_DEADBAND_BACKUP;

  private Pose2d finalTargetPose;

  private double finalVelocity;

  private boolean canDriveAimingAtPose = false;

  private String state;

  double distanceToTargetBranch;

  public GoToBranchConfiguration(double minErrorPose, double maxErrorPose,
      double errorForRotationAlignPose, double errorElevatorRaisedPose, String goToBranchMode, double minVelocity,
      double midVelocity, double maxVelocity) {
    this(false, true, true);
    this.minErrorPose = minErrorPose;
    this.maxErrorPose = maxErrorPose;
    this.errorElevatorRaisedPose = errorElevatorRaisedPose;
    this.goToBranchMode = goToBranchMode;
    this.midVelocity = midVelocity;
    this.minVelocity = minVelocity;
    this.maxVelocity = maxVelocity;
  }

  public GoToBranchConfiguration(double errorForRotationAlignPose,
      double minErrorPose, double midErrorPose, double maxErrorPose, String goToBranchMode, double minVelocity,
      double midVelocity, double maxVelocity, double ultraVelocity) {
    this(true, true, false);
    this.minErrorPose = minErrorPose;
    this.maxErrorPose = maxErrorPose;
    this.errorForRotationAlignPose = errorForRotationAlignPose;
    this.goToBranchMode = goToBranchMode;
    this.midVelocity = midVelocity;
    this.minVelocity = minVelocity;
    this.maxVelocity = maxVelocity;
    this.ultraVelocity = ultraVelocity;
  }

  public GoToBranchConfiguration(double minErrorPose, double maxErrorPose,
      double errorForRotationAlignPose, String goToBranchMode, double minVelocity, double midVelocity,
      double maxVelocity) {
    this(false, true, false);
    this.minErrorPose = minErrorPose;
    this.maxErrorPose = maxErrorPose;
    this.errorForRotationAlignPose = errorForRotationAlignPose;
    this.goToBranchMode = goToBranchMode;
    this.midVelocity = midVelocity;
    this.minVelocity = minVelocity;
    this.maxVelocity = maxVelocity;
  }

  public GoToBranchConfiguration(double minErrorPose, double midErrorPose,
      double maxErrorPose, double errorForRotationAlignPose, double errorElevatorRaisedPose, String goToBranchMode,
      double elevatorRaisedVelocity, double minVelocity, double midVelocity, double maxVelocity, double ultraVelocity) {
    this(true, true, true);
    this.minErrorPose = minErrorPose;
    this.midErrorPose = midErrorPose;
    this.maxErrorPose = maxErrorPose;
    this.errorForRotationAlignPose = errorForRotationAlignPose;
    this.errorElevatorRaisedPose = errorElevatorRaisedPose;
    this.goToBranchMode = goToBranchMode;
    this.midVelocity = midVelocity;
    this.minVelocity = minVelocity;
    this.maxVelocity = maxVelocity;
    this.ultraVelocity = ultraVelocity;
  }

  private GoToBranchConfiguration(boolean useMidError, boolean useRotationAlignPose,
      boolean useErrorElevatorRaisedPose) {
    this.useMidError = useMidError;
    this.useRotationAlignPose = useRotationAlignPose;
    this.useErrorElevatorRaisedPose = useErrorElevatorRaisedPose;
  }

  public void updateBranchData(Pose2d robotPose, Supplier<ReefLevel> scorerTargetReefLevelSupplier,
      Supplier<Boolean> elevatorAtHighPositionSupplier) {
    this.distanceToTargetBranch = branch.getTargetPoseToScore().getTranslation()
        .getDistance(robotPose.getTranslation());
    Pose2d targetBranchScorePose = scorerTargetReefLevelSupplier.get() == ReefLevel.L4
        ? CoordinatesTransform.getRetreatPose(branch.getTargetPoseToScore(), 0.05)
        : branch.getTargetPoseToScore();

    if (useErrorElevatorRaisedPose) {
      if (elevatorAtHighPositionSupplier.get() && distanceToTargetBranch < this.errorElevatorRaisedPose) {
        this.finalTargetPose = getDriveTarget(robotPose, targetBranchScorePose, this.backupBranch, this.backupBranch);
        this.finalVelocity = this.minVelocity;
        this.state = "DRIVE_TO_BRANCH_" + this.goToBranchMode + branch.name() + "_ELEVATOR_TOO_HIGH_AUTONOMOUS";
        if (useRotationAlignPose) {
          if (distanceToTargetBranch < errorForRotationAlignPose) {
            this.canDriveAimingAtPose = true;
            this.finalTargetPose = getDriveTarget(robotPose, targetBranchScorePose, backupBranch, this.backupBranch);
            this.finalVelocity = this.minVelocity;
            return;
          }
        }
        return;
      }
    }

    if (useMidError) {
      if (distanceToTargetBranch < this.maxErrorPose) {
        this.finalTargetPose = getDriveTarget(robotPose, targetBranchScorePose, backupBranch, false);
        this.finalVelocity = this.maxVelocity;
        this.state = "DRIVE_TO_BRANCH_" + this.goToBranchMode + branch.name() + "_CLOSE_AUTONOMOUS";
        if (distanceToTargetBranch < this.midErrorPose) {
          this.finalTargetPose = getDriveTarget(robotPose, targetBranchScorePose, backupBranch, false);
          this.finalVelocity = this.midVelocity;
          this.state = "DRIVE_TO_BRANCH_" + this.goToBranchMode + branch.name() + "_CLOSE_AUTONOMOUS";
          if (distanceToTargetBranch < this.minErrorPose) {
            this.finalTargetPose = getDriveTarget(robotPose, targetBranchScorePose, backupBranch, false);
            this.finalVelocity = this.minVelocity;
            if (distanceToTargetBranch < this.errorForRotationAlignPose) {
              this.canDriveAimingAtPose = true;
              this.finalTargetPose = getDriveTarget(robotPose, targetBranchScorePose, backupBranch, false);
              this.finalVelocity = this.minVelocity;
              return;
            }
          }
        }
        return;
      }
    } else {
      if (distanceToTargetBranch < this.maxErrorPose) {
        this.finalTargetPose = getDriveTarget(robotPose, targetBranchScorePose, backupBranch, false);
        this.finalVelocity = this.midVelocity;
        this.state = "DRIVE_TO_BRANCH_" + this.goToBranchMode + branch.name() + "_CLOSE_AUTONOMOUS";

        if (distanceToTargetBranch < this.errorForRotationAlignPose) {
          this.canDriveAimingAtPose = true;
          this.finalTargetPose = getDriveTarget(robotPose, targetBranchScorePose, backupBranch, false);
          return;
        }
        return;
      }
      this.finalTargetPose = getDriveTarget(robotPose, targetBranchScorePose, backupBranch, false);
      this.finalVelocity = this.maxVelocity;
    }
    this.finalTargetPose = getDriveTarget(robotPose, targetBranchScorePose, backupBranch, false);
    this.finalVelocity = this.ultraVelocity;
    this.state = "DRIVE_TO_BRANCH_" + this.goToBranchMode + branch.name() + "_FAR_AUTONOMOUS";
  }

  private Pose2d getDriveTarget(Pose2d robot, Pose2d goal, boolean moveBack, boolean goDirect) {
    if (moveBack) {
      goal = goal.transformBy(GeomUtil.toTransform2d(-0.25, 0.0));
      this.goToPoseTranslationDeadband = AutoAlignConstants.PoseDeadBand.POSE_TRANSLATION_DEADBAND_BACKUP;
      this.goToPoseHeadingDeadband = AutoAlignConstants.PoseDeadBand.POSE_HEADING_DEADBAND_BACKUP;
    } else {
      goal = goal.transformBy(GeomUtil.toTransform2d(-0.11, 0.0));
      this.goToPoseTranslationDeadband = AutoAlignConstants.PoseDeadBand.POSE_TRANSLATION_DEADBAND;
      this.goToPoseHeadingDeadband = AutoAlignConstants.PoseDeadBand.POSE_HEADING_DEADBAND;
    }
    if (goDirect) {
      return goal;
    }
    return this.calculateReefAvoidenceTarget(robot, goal);
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

  public double getPoseTranslationDeadband() {
    return this.goToPoseTranslationDeadband;
  }

  public double getPoseHeadingDeadband() {
    return this.goToPoseHeadingDeadband;
  }

  public String getGoToBranchState() {
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

  public void setBranch(TargetBranch targetBranch, boolean backupBranch) {
    this.branch = targetBranch;
    this.backupBranch = backupBranch;
  }

  public double getDistanceToTargetBranch() {
    return this.distanceToTargetBranch;
  }

}
