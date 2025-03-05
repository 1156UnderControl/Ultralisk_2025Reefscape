package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.Java_Is_UnderControl.Control.PIDConfig;
import frc.Java_Is_UnderControl.Util.AllianceFlipUtil;
import frc.Java_Is_UnderControl.Util.CoordinatesTransform;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.ReefLevel;

public class SwerveConstants {
  public static final PIDConfig MOVE_TO_POSE_TRANSLATION_PID = new PIDConfig(3.5, 0, 0, 0, 0);
  public static final Constraints MOVE_TO_POSE_TRANSLATION_CONSTRAINTS = new Constraints(4, 3);
  public static final Constraints MOVE_TO_POSE_X_CONSTRAINTS = new Constraints(4, 3);
  public static final Constraints MOVE_TO_POSE_Y_CONSTRAINTS = new Constraints(4, 3);

  public enum TargetBranch {
    A(1), B(0), C(11), D(10), E(9), F(9), G(7), H(6), I(5), J(4), K(3), L(2);

    private final int branchIndex;

    TargetBranch(int branchIndex) {
      this.branchIndex = branchIndex;
    }

    public Pose2d getTargetPoseToScore() {
      Pose2d transformedPose = CoordinatesTransform.applyRotationToPoseAngle(
          CoordinatesTransform
              .getRetreatPose(AllianceFlipUtil.apply(Reef.branchPositions2d.get(branchIndex).get(ReefLevel.L4)), 0.6),
          Rotation2d.k180deg);
      return transformedPose;
    }
  }

  public enum PoseEstimatorState {
    REEF_ESTIMATION, GLOBAL_POSE_ESTIMATION, AUTO_POSE_ESTIMATION;
  }

  public class CoralStations {
    public static final Pose2d CORAL_STATION_RIGHT_POSE_FOR_ROBOT = new Pose2d(
        Units.inchesToMeters(33.526),
        Units.inchesToMeters(25.824),
        Rotation2d.fromDegrees(45)).rotateBy(Rotation2d.k180deg);

    public static final Pose2d CORAL_STATION_LEFT_POSE_FOR_ROBOT = new Pose2d(
        Units.inchesToMeters(33.526),
        Units.inchesToMeters(291.176),
        Rotation2d.fromDegrees(-45));
  }
}
