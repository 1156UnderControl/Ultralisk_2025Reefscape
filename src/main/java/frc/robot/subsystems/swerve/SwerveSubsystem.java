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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Java_Is_UnderControl.Control.PIDConfig;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomStringLogger;
import frc.Java_Is_UnderControl.Swerve.MoveToPosePIDConfig;
import frc.Java_Is_UnderControl.Swerve.OdometryEnabledSwerveConfig;
import frc.Java_Is_UnderControl.Swerve.OdometryEnabledSwerveSubsystem;
import frc.Java_Is_UnderControl.Swerve.SwervePathPlannerConfig;
import frc.Java_Is_UnderControl.Util.AllianceFlipUtil;
import frc.Java_Is_UnderControl.Util.GeomUtil;
import frc.Java_Is_UnderControl.Util.StabilizeChecker;
import frc.Java_Is_UnderControl.Vision.Deprecated.Cameras.LimelightHelpers;
import frc.Java_Is_UnderControl.Vision.Odometry.LimelightPoseEstimator;
import frc.Java_Is_UnderControl.Vision.Odometry.MultiCameraPoseEstimator;
import frc.Java_Is_UnderControl.Vision.Odometry.NoPoseEstimator;
import frc.Java_Is_UnderControl.Vision.Odometry.PhotonVisionPoseEstimator;
import frc.Java_Is_UnderControl.Vision.Odometry.PoseEstimator;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.SwerveConstants.PoseEstimatorState;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.constants.VisionConstants;
import frc.robot.joysticks.DriverController;
import frc.robot.pose_estimators.ReefPoseEstimator;

public class SwerveSubsystem extends OdometryEnabledSwerveSubsystem implements ISwerve {

  private DriverController controller = DriverController.getInstance();

  private Matrix<N3, N1> defaultVisionStandardDeviation;

  private Matrix<N3, N1> defaultOdometryStandardDeviation;

  private TargetBranch targetBranch = TargetBranch.A;

  private double goToPoseTranslationDeadband = 0.02;

  private double goToPoseHeadingDeadband = 3;

  private static PhotonCamera arducamLeft = new PhotonCamera("Arducam-left");

  private static PhotonCamera arducamRight = new PhotonCamera("Arducam-right");

  private ReefPoseEstimator reefPoseEstimator = new ReefPoseEstimator(arducamLeft,
      VisionConstants.robotToCamArducamLeft,
      arducamRight, VisionConstants.robotToCamArducamRight);

  CustomStringLogger swerveStateLogger = new CustomStringLogger("SwerveSubsystem/State");

  private StabilizeChecker stableAtTargetPose = new StabilizeChecker(0.5);

  private String state = "NULL";

  CustomStringLogger poseEstimatorStateLogger = new CustomStringLogger("SwerveSubsystem/State_PoseEstimator");

  private PoseEstimatorState poseEstimatorState = PoseEstimatorState.GLOBAL_POSE_ESTIMATION;

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
        new MoveToPosePIDConfig(SwerveConstants.MOVE_TO_POSE_TRANSLATION_PID,
            SwerveConstants.MOVE_TO_POSE_Y_CONSTRAINTS)),
        drivetrainConstants,
        modules);
  }

  private static PoseEstimator configureMulticameraPoseEstimation() {
    List<PoseEstimator> listOfEstimators = new ArrayList<PoseEstimator>();
    PoseEstimator arducamRightEstimator = new PhotonVisionPoseEstimator(arducamRight,
        VisionConstants.robotToCamArducamRight, false);
    PoseEstimator arducamLeftEstimator = new PhotonVisionPoseEstimator(arducamLeft,
        VisionConstants.robotToCamArducamLeft,
        false);
    PoseEstimator limelightReef = new LimelightPoseEstimator("limelight-reef", false, true, 2);
    PoseEstimator limelightSource = new LimelightPoseEstimator("limelight-source", false, true, 2);
    listOfEstimators.add(arducamRightEstimator);
    listOfEstimators.add(arducamLeftEstimator);
    listOfEstimators.add(limelightReef);
    listOfEstimators.add(limelightSource);
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
    this.state = "PATHFIND_TO_POSE";
    return driveToPoseWithPathfinding(pose.toPose2d());
  }

  public void setTargetBranch(TargetBranch targetBranch) {
    this.targetBranch = targetBranch;
  }

  @Override
  public void periodic() {
    super.periodic();
    selectPoseEstimator();
    updateLogs();
    LimelightHelpers.SetRobotOrientation("limelight-reef",
        OdometryEnabledSwerveSubsystem.robotOrientation,
        OdometryEnabledSwerveSubsystem.robotAngularVelocity, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-source",
        OdometryEnabledSwerveSubsystem.robotOrientation,
        OdometryEnabledSwerveSubsystem.robotAngularVelocity, 0, 0, 0, 0);
  }

  private void selectPoseEstimator() {
    if (getPose().getTranslation().getDistance(AllianceFlipUtil.apply(FieldConstants.Reef.center)) < 2) {
      poseEstimatorState = PoseEstimatorState.REEF_ESTIMATION;
    } else {
      poseEstimatorState = PoseEstimatorState.GLOBAL_POSE_ESTIMATION;
    }
    switch (poseEstimatorState) {
      case GLOBAL_POSE_ESTIMATION:
        overrideTeleOpPoseEstimator(null);
        break;
      case REEF_ESTIMATION:
        overrideTeleOpPoseEstimator(reefPoseEstimator);
        break;
      default:
        overrideTeleOpPoseEstimator(null);
        break;
    }
  }

  protected void updateLogs() {
    this.swerveStateLogger.append(this.state);
  }

  @Override
  public void driveToBranch(TargetBranch branch, boolean backupBranch) {
    this.targetBranch = branch;
    double distanceToTargetBranch = targetBranch.getTargetPoseToScore().getTranslation()
        .getDistance(getPose().getTranslation());
    if (distanceToTargetBranch < 3) {
      if (distanceToTargetBranch < 1) {
        driveToPose(getDriveTarget(getPose(), targetBranch.getTargetPoseToScore(), backupBranch), 1);
        this.state = "DRIVE_TO_BRANCH_" + branch.name() + "_CLOSE";
        return;
      }
      driveToPose(getDriveTarget(getPose(), targetBranch.getTargetPoseToScore(), backupBranch), 2);
      this.state = "DRIVE_TO_BRANCH_" + branch.name() + "_FAR";
    } else {
      driveAlignAngleJoy();
    }
  }

  private Pose2d getNearestCoralStationPose() {
    if (this.getPose().getY() >= 4.0259) {
      return SwerveConstants.CoralStations.CORAL_STATION_RIGHT_POSE_FOR_ROBOT;
    } else {
      return SwerveConstants.CoralStations.CORAL_STATION_LEFT_POSE_FOR_ROBOT;
    }
  }

  @Override
  public void driveLockedAngleToNearestCoralStation() {
    Rotation2d nearestCoralStationRotationAngle = this.getNearestCoralStationPose().getRotation();

    ChassisSpeeds desiredSpeeds = this.inputsToChassisSpeeds(controller.getYtranslation(),
        controller.getXtranslation());
    this.state = "DRIVE_ALIGN_ANGLE_CORAL_STATION";
    this.driveFieldOrientedLockedJoystickAngle(desiredSpeeds, nearestCoralStationRotationAngle.getCos(),
        nearestCoralStationRotationAngle.getSin());
  }

  @Override
  public void driveToNearestCoralStation() {
    Pose2d nearestCoralStationPose2D = this.getNearestCoralStationPose();
    this.driveToPose(nearestCoralStationPose2D);
  }

  @Override
  public boolean isAtTargetPosition() {
    return stableAtTargetPose.isStableInCondition(() -> isAtTargetPose(this.goToPoseTranslationDeadband,
        this.goToPoseTranslationDeadband, this.goToPoseHeadingDeadband));
  }

  public void driveToPoseTest() {
    driveToPose(new Pose2d(15, 2, new Rotation2d(Units.degreesToRadians(180))));
  }

  private Pose2d getDriveTarget(Pose2d robot, Pose2d goal, boolean moveBack) {
    if (moveBack) {
      goal = goal.transformBy(GeomUtil.toTransform2d(-0.25, 0.0));
      this.goToPoseTranslationDeadband = 0.05;
      this.goToPoseHeadingDeadband = 10;
    } else {
      goal = goal.transformBy(GeomUtil.toTransform2d(-0.11, 0.0));
      this.goToPoseTranslationDeadband = 0.025;
      this.goToPoseHeadingDeadband = 3;
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
