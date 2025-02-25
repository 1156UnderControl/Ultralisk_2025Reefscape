package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Java_Is_UnderControl.Control.PIDConfig;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomPose2dLogger;
import frc.Java_Is_UnderControl.Swerve.MoveToPosePIDConfig;
import frc.Java_Is_UnderControl.Swerve.OdometryEnabledSwerveConfig;
import frc.Java_Is_UnderControl.Swerve.OdometryEnabledSwerveSubsystem;
import frc.Java_Is_UnderControl.Swerve.SwervePathPlannerConfig;
import frc.Java_Is_UnderControl.Util.GeomUtil;
import frc.Java_Is_UnderControl.Vision.Deprecated.Cameras.LimelightHelpers;
import frc.Java_Is_UnderControl.Vision.Odometry.MultiCameraPoseEstimator;
import frc.Java_Is_UnderControl.Vision.Odometry.NoPoseEstimator;
import frc.Java_Is_UnderControl.Vision.Odometry.PhotonVisionPoseEstimator;
import frc.Java_Is_UnderControl.Vision.Odometry.PoseEstimator;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.joysticks.DriverController;

public class SwerveSubsystem extends OdometryEnabledSwerveSubsystem implements ISwerve {

  private DriverController controller = DriverController.getInstance();

  private Matrix<N3, N1> defaultVisionStandardDeviation;

  private Matrix<N3, N1> defaultOdometryStandardDeviation;

  private TargetBranch targetBranch = TargetBranch.A;

  CustomPose2dLogger posetraj = new CustomPose2dLogger("Testpose");

  private String state = "NULL";

  private static final SwervePathPlannerConfig pathPlannerConfig = new SwervePathPlannerConfig(
      new PIDConstants(5, 0, 0),
      new PIDConstants(5, 0, 0),
      new PathConstraints(
          3.0, 4.0,
          Units.degreesToRadians(540), Units.degreesToRadians(720)));

  public SwerveSubsystem(
      SwerveDrivetrainConstants drivetrainConstants,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(new OdometryEnabledSwerveConfig(0.75, pathPlannerConfig,
        new NoPoseEstimator(),
        configureMulticameraPoseEstimation(),
        new PIDConfig(7.1, 0, 0.06),
        new MoveToPosePIDConfig(SwerveConstants.MOVE_TO_POSE_X_PID, SwerveConstants.MOVE_TO_POSE_X_CONSTRAINTS,
            SwerveConstants.MOVE_TO_POSE_Y_PID, SwerveConstants.MOVE_TO_POSE_Y_CONSTRAINTS)),
        drivetrainConstants,
        modules);
  }

  private static PoseEstimator configureMulticameraPoseEstimation() {
    Transform3d robotToCamArducamLeft = new Transform3d(new Translation3d(-0.2613, 0.210, 0.1966),
        new Rotation3d(0, Units.degreesToRadians(-18.125), Units.degreesToRadians(30)));
    Transform3d robotToCamArducamRight = new Transform3d(new Translation3d(0.2613, 0.210, 0.1966),
        new Rotation3d(0, Units.degreesToRadians(-18.125), Units.degreesToRadians(-30)));
    List<PoseEstimator> listOfEstimators = new ArrayList<>();
    PoseEstimator arducamRight = new PhotonVisionPoseEstimator(new PhotonCamera("Arducam-right"),
        robotToCamArducamRight, false);
    PoseEstimator arducamLeft = new PhotonVisionPoseEstimator(new PhotonCamera("Arducam-left"), robotToCamArducamLeft,
        false);
    listOfEstimators.add(arducamRight);
    listOfEstimators.add(arducamLeft);
    PoseEstimator estimatorMultiCamera = new MultiCameraPoseEstimator(listOfEstimators);
    return estimatorMultiCamera;
  }

  public void driveAlignAngleJoy() {
    ChassisSpeeds desiredSpeeds = this.inputsToChassisSpeeds(controller.getYtranslation(),
        controller.getXtranslation());
    this.state = "DRIVE_ALIGN_ANGLE_JOY";
    this.driveFieldOrientedLockedJoystickAngle(desiredSpeeds, controller.getCOS_Joystick(),
        controller.getSIN_Joystick());
  }

  public void driveRotatingButton() {
    ChassisSpeeds desiredSpeeds = this.inputsToChassisSpeeds(controller.getYtranslation(),
        controller.getXtranslation(), 0);
    double angleTarget = Math.atan2(controller.getCOS_Joystick(), controller.getSIN_Joystick());
    this.state = "DRIVE_ALIGN_ANGLE_JOY";
    this.driveFieldOrientedLockedAngle(desiredSpeeds, new Rotation2d(angleTarget));
  }

  public Command goToPoseWithPathfind(Pose2d pose) {
    return driveToPoseWithPathfinding(pose);
  }

  public Command goToPoseWithPathfind(Pose3d pose) {

    return driveToPoseWithPathfinding(pose.toPose2d());
  }

  public void setTargetBranch(TargetBranch targetBranch) {
    this.targetBranch = targetBranch;
  }

  @Override
  public void periodic() {
    super.periodic();
    Pose2d currentPose = SwerveConstants.TargetBranch.B.getTargetPoseToScore();
    posetraj.appendRadians(getDriveTarget(getPose(), currentPose, false));
    LimelightHelpers.SetRobotOrientation("limelight-reef",
        OdometryEnabledSwerveSubsystem.robotOrientation,
        OdometryEnabledSwerveSubsystem.robotAngularVelocity, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-source",
        OdometryEnabledSwerveSubsystem.robotOrientation,
        OdometryEnabledSwerveSubsystem.robotAngularVelocity, 0, 0, 0, 0);
  }

  @Override
  protected void updateLogs() {

  }

  @Override
  public void driveAimingToNearestHP() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'driveAimingToNearestHP'");
  }

  /** Get drive target. */
  private static Pose2d getDriveTarget(Pose2d robot, Pose2d goal, boolean moveBack) {
    // If superstructure isn't ready move back away from reef
    if (moveBack) {
      goal = goal.transformBy(GeomUtil.toTransform2d(-0.5, 0.0));
    }

    // Final line up
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

}

  