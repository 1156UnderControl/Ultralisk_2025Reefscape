package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Java_Is_UnderControl.Control.PIDConfig;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomPose2dLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomStringLogger;
import frc.Java_Is_UnderControl.Swerve.MoveToPosePIDConfig;
import frc.Java_Is_UnderControl.Swerve.OdometryEnabledSwerveConfig;
import frc.Java_Is_UnderControl.Swerve.OdometryEnabledSwerveSubsystem;
import frc.Java_Is_UnderControl.Swerve.SwervePathPlannerConfig;
import frc.Java_Is_UnderControl.Util.AllianceFlipUtil;
import frc.Java_Is_UnderControl.Util.CoordinatesTransform;
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
import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.SwerveConstants.PoseEstimatorState;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.constants.VisionConstants;
import frc.robot.joysticks.DriverController;
import frc.robot.pose_estimators.ReefPoseEstimatorWithLimelight;

public class SwerveSubsystem extends OdometryEnabledSwerveSubsystem implements ISwerve {

  private DriverController controller = DriverController.getInstance();

  private Matrix<N3, N1> defaultVisionStandardDeviation;

  private Matrix<N3, N1> defaultOdometryStandardDeviation;

  private TargetBranch targetBranch = TargetBranch.A;

  private double goToPoseTranslationDeadband = 0.03;

  private double goToPoseHeadingDeadband = 1;

  Supplier<ReefLevel> scorerTargetReefLevelSupplier;

  Supplier<Boolean> elevatorAtHighPositionSupplier;

  CustomPose2dLogger logPoses = new CustomPose2dLogger("pose reef");

  private static PhotonCamera arducamLeft = new PhotonCamera("Arducam-left");

  private static PhotonCamera arducamRight = new PhotonCamera("Arducam-right");

  private ReefPoseEstimatorWithLimelight reefPoseEstimator = new ReefPoseEstimatorWithLimelight("limelight-reef",
      arducamRight, VisionConstants.robotToCamArducamRight, () -> getTargetBranch());

  CustomStringLogger swerveStateLogger = new CustomStringLogger("SwerveSubsystem/State");

  private StabilizeChecker stableAtTargetPose = new StabilizeChecker(0.150);

  private String state = "NULL";

  CustomStringLogger poseEstimatorStateLogger = new CustomStringLogger("SwerveSubsystem/State_PoseEstimator");

  private PoseEstimatorState poseEstimatorState = PoseEstimatorState.GLOBAL_POSE_ESTIMATION;

  private boolean forceReefPoseEstimation = false;

  private Rotation2d bestAngleForClimb = new Rotation2d();

  private double distanceToTargetBranch = Double.POSITIVE_INFINITY;

  private static final SwervePathPlannerConfig pathPlannerConfig = new SwervePathPlannerConfig(
      new PIDConstants(5, 0, 0),
      new PIDConstants(5, 0, 0),
      new PathConstraints(
          3.0, 4.0,
          Units.degreesToRadians(540), Units.degreesToRadians(720)));

  public SwerveSubsystem(Supplier<Boolean> elevatorAtHighPositionSupplier, Supplier<ReefLevel> scorerTargetReefLevel,
      SwerveDrivetrainConstants drivetrainConstants,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(new OdometryEnabledSwerveConfig(0.75, pathPlannerConfig,
        new NoPoseEstimator(),
        configureMulticameraPoseEstimation(),
        new PIDConfig(7.5, 0, 0.3),
        new MoveToPosePIDConfig(SwerveConstants.MOVE_TO_POSE_TRANSLATION_PID,
            SwerveConstants.MOVE_TO_POSE_Y_CONSTRAINTS)),
        drivetrainConstants,
        modules);
    this.elevatorAtHighPositionSupplier = elevatorAtHighPositionSupplier;
    this.scorerTargetReefLevelSupplier = scorerTargetReefLevel;
  }

  private static PoseEstimator configureMulticameraPoseEstimation() {
    List<PoseEstimator> listOfEstimators = new ArrayList<PoseEstimator>();
    PoseEstimator arducamRightEstimator = new PhotonVisionPoseEstimator(arducamRight,
        VisionConstants.robotToCamArducamRight, false);
    PoseEstimator arducamLeftEstimator = new PhotonVisionPoseEstimator(arducamLeft,
        VisionConstants.robotToCamArducamLeft,
        false);
    PoseEstimator limelightReef = new LimelightPoseEstimator("limelight-reef", false, false, 2);
    PoseEstimator limelightSource = new LimelightPoseEstimator("limelight-source", false, false, 2);
    listOfEstimators.add(arducamRightEstimator);
    listOfEstimators.add(arducamLeftEstimator);
    listOfEstimators.add(limelightReef);
    listOfEstimators.add(limelightSource);
    PoseEstimator estimatorMultiCamera = new MultiCameraPoseEstimator(listOfEstimators, "Teleop Multi Pose Estimator");
    return estimatorMultiCamera;
  }

  @Override
  public void driveAlignAngleJoystick() {
    if (elevatorAtHighPositionSupplier.get()) {
      driveAlignAngleJoystickSuperSlow();
      return;
    }
    if (controller.leftBumper().getAsBoolean()) {
      driveRotating(false);
      return;
    }
    if (controller.rightBumper().getAsBoolean()) {
      driveRotating(true);
      return;
    }
    ChassisSpeeds desiredSpeeds = this.inputsToChassisSpeeds(controller.getYtranslation(),
        controller.getXtranslation());
    this.state = "DRIVE_ALIGN_ANGLE_JOY";
    this.driveFieldOrientedLockedJoystickAngle(desiredSpeeds, controller.getCOS_Joystick(),
        controller.getSIN_Joystick());
  }

  public void driveRotating(boolean rotateRight) {
    ChassisSpeeds desiredSpeeds = this.inputsToChassisSpeeds(controller.getYtranslation(),
        controller.getXtranslation(), rotateRight ? -1 : 1);
    this.state = "DRIVE_ALIGN_ANGLE_ROTATING_RIGHT?:" + Boolean.toString(rotateRight);
    this.driveFieldOriented(desiredSpeeds);
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
    selectPoseEstimator();
    super.periodic();
    updateLogs();
    reefPoseEstimator.setHeadingMeasurement(getHeading());
    LimelightHelpers.SetRobotOrientation("limelight-reef",
        OdometryEnabledSwerveSubsystem.robotOrientation,
        OdometryEnabledSwerveSubsystem.robotAngularVelocity, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-source",
        OdometryEnabledSwerveSubsystem.robotOrientation,
        OdometryEnabledSwerveSubsystem.robotAngularVelocity, 0, 0, 0, 0);
  }

  private void selectPoseEstimator() {
    if ((getPose().getTranslation().getDistance(AllianceFlipUtil.apply(FieldConstants.Reef.center)) < 2
        && (state.contains("DRIVE_TO_BRANCH") || state.contains("STOP")) || this.forceReefPoseEstimation)) {
      poseEstimatorState = PoseEstimatorState.REEF_ESTIMATION;
    } else {
      poseEstimatorState = PoseEstimatorState.GLOBAL_POSE_ESTIMATION;
    }
    switch (poseEstimatorState) {
      case GLOBAL_POSE_ESTIMATION:
        overrideTeleOpPoseEstimator(null);
        overrideAutonomousPoseEstimator(null);
        break;
      case REEF_ESTIMATION:
        overrideTeleOpPoseEstimator(reefPoseEstimator);
        overrideAutonomousPoseEstimator(reefPoseEstimator);
        break;
      default:
        overrideTeleOpPoseEstimator(null);
        overrideAutonomousPoseEstimator(null);
        break;
    }
  }

  public void forceReefPoseEstimation(boolean forceReefPoseEstimation) {
    this.forceReefPoseEstimation = forceReefPoseEstimation;
  }

  protected void updateLogs() {
    this.swerveStateLogger.append(this.state);
  }

  @Override
  public void driveToBranch(TargetBranch branch, boolean backupBranch) {
    this.targetBranch = branch;
    if (DriverStation.isAutonomousEnabled()) {
      goToBranchAutonomous(branch, backupBranch);
      return;
    }
    goToBranchTeleoperated(branch, backupBranch);
  }

  private void goToBranchTeleoperated(TargetBranch branch, boolean backupBranch) {
    this.distanceToTargetBranch = targetBranch.getTargetPoseToScore().getTranslation()
        .getDistance(getPose().getTranslation());
    Pose2d targetBranchScorePose = this.scorerTargetReefLevelSupplier.get() == ReefLevel.L4
        ? CoordinatesTransform.getRetreatPose(targetBranch.getTargetPoseToScore(), 0.05)
        : targetBranch.getTargetPoseToScore();
    if (this.distanceToTargetBranch < 3) {
      if (this.distanceToTargetBranch < 1) {
        driveToPose(getDriveTarget(getPose(), targetBranchScorePose, backupBranch), 1);
        this.state = "DRIVE_TO_BRANCH_" + branch.name() + "_CLOSE";
        return;
      }
      driveToPose(getDriveTarget(getPose(), targetBranchScorePose, backupBranch), 2);
      this.state = "DRIVE_TO_BRANCH_" + branch.name() + "_FAR";
    } else {
      driveAlignAngleJoystick();
    }
  }

  private void goToBranchAutonomous(TargetBranch branch, boolean backupBranch) {
    this.distanceToTargetBranch = targetBranch.getTargetPoseToScore().getTranslation()
        .getDistance(getPose().getTranslation());
    Pose2d targetBranchScorePose = this.scorerTargetReefLevelSupplier.get() == ReefLevel.L4
        ? CoordinatesTransform.getRetreatPose(targetBranch.getTargetPoseToScore(), 0.05)
        : targetBranch.getTargetPoseToScore();

    if (elevatorAtHighPositionSupplier.get()) {
      driveToPose(getDriveTarget(getPose(), targetBranchScorePose, backupBranch), 0.7);
      this.state = "DRIVE_TO_BRANCH_" + branch.name() + "_ELEVATOR_TOO_HIGH_AUTONOMOUS";
      return;
    }

    if (this.distanceToTargetBranch < 2) {
      driveToPose(getDriveTarget(getPose(), targetBranchScorePose, backupBranch), 1.5);
      this.state = "DRIVE_TO_BRANCH_" + branch.name() + "_CLOSE_AUTONOMOUS";
      return;
    }
    driveToPose(getDriveTarget(getPose(), targetBranchScorePose, backupBranch), 3.5);
    this.state = "DRIVE_TO_BRANCH_" + branch.name() + "_FAR_AUTONOMOUS";
  }

  public double getDistanceToTargetBranch() {
    return this.distanceToTargetBranch;
  }

  private TargetBranch getTargetBranch() {
    return this.targetBranch;
  }

  private Pose2d getNearestCoralStationPose() {
    if (this.getPose().getY() >= 4.0259) {
      return SwerveConstants.CoralStations.CORAL_STATION_RIGHT_POSE_FOR_ROBOT;
    } else {
      return SwerveConstants.CoralStations.CORAL_STATION_LEFT_POSE_FOR_ROBOT;
    }
  }

  public void setAngleForClimb() {
    if (this.getPose().getX() >= FieldConstants.fieldLength / 2) {
      this.bestAngleForClimb = AllianceFlipUtil.apply(Rotation2d.fromDegrees(-90));
    } else {
      this.bestAngleForClimb = AllianceFlipUtil.apply(Rotation2d.fromDegrees(90));
    }
  }

  @Override
  public void driveLockedAngleToNearestCoralStation() {
    if (!controller.notUsingJoystick()) {
      this.driveAlignAngleJoystick();
      return;
    }
    Rotation2d nearestCoralStationRotationAngle = this.getNearestCoralStationPose().getRotation();
    ChassisSpeeds desiredSpeeds = this.inputsToChassisSpeeds(controller.getYtranslation(),
        controller.getXtranslation());
    this.state = "DRIVE_ALIGN_ANGLE_CORAL_STATION";
    if (!controller.notUsingJoystick()) {
      this.driveAlignAngleJoystick();
    } else {
      this.driveFieldOrientedLockedJoystickAngle(desiredSpeeds, nearestCoralStationRotationAngle.getCos(),
          nearestCoralStationRotationAngle.getSin());
    }
  }

  @Override
  public void driveLockedAngleToClimb() {
    if (!controller.notUsingJoystick()) {
      this.driveAlignAngleJoystick();
      return;
    }
    ChassisSpeeds desiredSpeeds = this.inputsToChassisSpeeds(controller.getYtranslation(),
        controller.getXtranslation());
    this.state = "DRIVE_ALIGN_ANGLE_CLIMB";
    this.driveFieldOrientedLockedAngle(desiredSpeeds.times(0.5), this.bestAngleForClimb);
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

  @Override
  public void stopSwerve() {
    this.driveRobotOriented(new ChassisSpeeds());
    this.state = "STOP_SWERVE";
  }

  @Override
  public void driveAlignAngleJoystickSuperSlow() {
    ChassisSpeeds desiredSpeeds = this.inputsToChassisSpeeds(controller.getYtranslation(),
        controller.getXtranslation()).times(0.12);
    this.state = "DRIVE_ALIGN_ANGLE_JOY_SUPER_SLOW";
    this.driveFieldOrientedLockedJoystickAngle(desiredSpeeds, controller.getCOS_Joystick(),
        controller.getSIN_Joystick());
  }

  public void driveAlignAngleJoystickRemoveAlgae() {
    ChassisSpeeds desiredSpeeds = this.inputsToChassisSpeeds(controller.getYtranslation(),
        controller.getXtranslation()).times(0.3);
    this.state = "DRIVE_ALIGN_ANGLE_JOY_REMOVE_ALGAE";
    this.driveFieldOrientedLockedJoystickAngle(desiredSpeeds, controller.getCOS_Joystick(),
        controller.getSIN_Joystick());
  }

  @Override
  public boolean swerveIsToCloseToReefForLiftingElevador() {
    return getPose().getTranslation().getDistance(AllianceFlipUtil.apply(Reef.center)) < 1;
  }

}
