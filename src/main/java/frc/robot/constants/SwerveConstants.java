package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.Java_Is_UnderControl.Control.PIDConfig;
import frc.Java_Is_UnderControl.Util.AllianceFlipUtil;
import frc.Java_Is_UnderControl.Util.CoordinatesTransform;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.ReefLevel;

public class SwerveConstants {
  public static final PIDConfig MOVE_TO_POSE_TRANSLATION_PID = new PIDConfig(6.5, 2, 0, 0, 0.5);
  public static final PIDConfig MOVE_TO_POSE_X_PID = new PIDConfig(8, 3.5, 0, 0, 0.2);
  public static final PIDConfig MOVE_TO_POSE_Y_PID = new PIDConfig(1.5, 3, 0, 0, 1);
  public static final Constraints MOVE_TO_POSE_TRANSLATION_CONSTRAINTS = new Constraints(4, 3);
  public static final Constraints MOVE_TO_POSE_X_CONSTRAINTS = new Constraints(4, 3);
  public static final Constraints MOVE_TO_POSE_Y_CONSTRAINTS = new Constraints(4, 3);

  public enum TargetBranch {
    A(1), B(0), C(12), D(11), E(10), F(9), G(8), H(7), I(6), J(5), K(4), L(3), M(2);

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
}
