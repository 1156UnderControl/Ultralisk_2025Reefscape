package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Java_Is_UnderControl.Control.PIDConfig;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomBooleanLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomStringLogger;
import frc.Java_Is_UnderControl.Swerve.MoveToPosePIDConfig;
import frc.Java_Is_UnderControl.Swerve.OdometryEnabledSwerveConfig;
import frc.Java_Is_UnderControl.Swerve.OdometryEnabledSwerveSubsystem;
import frc.Java_Is_UnderControl.Swerve.SwervePathPlannerConfig;
import frc.Java_Is_UnderControl.Util.AllianceFlipUtil;
import frc.Java_Is_UnderControl.Util.StabilizeChecker;
import frc.Java_Is_UnderControl.Vision.Deprecated.Cameras.LimelightHelpers;
import frc.Java_Is_UnderControl.Vision.Odometry.LimelightPoseEstimator;
import frc.Java_Is_UnderControl.Vision.Odometry.MultiCameraPoseEstimator;
import frc.Java_Is_UnderControl.Vision.Odometry.PoseEstimation;
import frc.Java_Is_UnderControl.Vision.Odometry.PoseEstimator;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Algae.AlgaeHeightReef;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.SwerveConstants.AutoAlignConstants;
import frc.robot.constants.SwerveConstants.PoseEstimatorState;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.constants.SwerveConstants.TargetFace;
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

  Supplier<AlgaeHeightReef> scorerTargetReefLevelAlgaeSupplier;

  Supplier<Boolean> elevatorAtHighPositionSupplier;

  int[] apriltagsIDs = new int[] { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22 };

  private ReefPoseEstimatorWithLimelight reefPoseEstimator = new ReefPoseEstimatorWithLimelight("limelight-left",
      "limelight-right", () -> getTargetBranch());

  CustomStringLogger swerveStateLogger = new CustomStringLogger("SwerveSubsystem/State");

  CustomBooleanLogger forcingReefPoseEstimatorLogger = new CustomBooleanLogger(
      "SwerveSubsystem/forcingReefPoseEstimatorLogger");

  private StabilizeChecker stableAtTargetPose = new StabilizeChecker(0.150);

  private String state = "NULL";

  CustomStringLogger poseEstimatorStateLogger = new CustomStringLogger("SwerveSubsystem/State_PoseEstimator");

  CustomStringLogger targetBranchLogger = new CustomStringLogger("SwerveSubsystem/Target Branch");

  CustomBooleanLogger isUsingAngleCorrection = new CustomBooleanLogger("SwerveSubsystem/UsingAngleCorrection");

  CustomDoubleLogger targetVelocity = new CustomDoubleLogger("SwerveSubsystem/TargetVelocity");

  CustomDoubleLogger distanceToTargetBranchLog = new CustomDoubleLogger("SwerveSubsystem/DistanceToTargetBranch");

  CustomDoubleLogger distanceToTargetFaceLog = new CustomDoubleLogger("SwerveSubsystem/DistanceToTargetFace");

  private PoseEstimatorState poseEstimatorState = PoseEstimatorState.GLOBAL_POSE_ESTIMATION;

  private boolean forceReefPoseEstimation = false;

  private Rotation2d bestAngleForClimb = new Rotation2d();

  private double distanceToTargetBranch = Double.POSITIVE_INFINITY;

  private double distanceToTargetFace = Double.POSITIVE_INFINITY;

  private boolean positionUpdated = false;

  private GoToBranchConfiguration goToBranchConfigurationFast;

  private GoToBranchConfiguration goToBranchConfigurationTeleoperated;

  private Pose2d targetFacePose = TargetFace.A.getTargetPoseToScore();

  private static final SwervePathPlannerConfig pathPlannerConfig = new SwervePathPlannerConfig(
      new PIDConstants(5, 0, 0),
      new PIDConstants(5, 0, 0),
      new PathConstraints(
          3.0, 4.0,
          Units.degreesToRadians(540), Units.degreesToRadians(720)));

  public SwerveSubsystem(Supplier<Boolean> elevatorAtHighPositionSupplier, Supplier<ReefLevel> scorerTargetReefLevel,
      Supplier<AlgaeHeightReef> scorerTargetReefLevelAlgae,
      SwerveDrivetrainConstants drivetrainConstants,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(new OdometryEnabledSwerveConfig(0.75, pathPlannerConfig,
        configureMulticameraPoseEstimation(),
        configureMulticameraPoseEstimation(),
        new PIDConfig(6, 0, 0),
        new MoveToPosePIDConfig(SwerveConstants.MOVE_TO_POSE_TRANSLATION_PID,
            SwerveConstants.MOVE_TO_POSE_Y_CONSTRAINTS)),
        drivetrainConstants,
        modules);
    this.elevatorAtHighPositionSupplier = elevatorAtHighPositionSupplier;
    this.scorerTargetReefLevelSupplier = scorerTargetReefLevel;
    this.scorerTargetReefLevelAlgaeSupplier = scorerTargetReefLevelAlgae;
    this.configureGoToBranch();
    this.getPigeon2().setYaw(0);
    this.resetOdometry(new Pose2d());
  }

  private static PoseEstimator configureMulticameraPoseEstimation() {
    List<PoseEstimator> listOfEstimators = new ArrayList<PoseEstimator>();
    PoseEstimator limelightLeft = new LimelightPoseEstimator("limelight-left", false, false, 2);
    PoseEstimator limelightRight = new LimelightPoseEstimator("limelight-right",
        false, false, 2);
    listOfEstimators.add(limelightLeft);
    listOfEstimators.add(limelightRight);
    PoseEstimator estimatorMultiCamera = new MultiCameraPoseEstimator(listOfEstimators, "Teleop Multi Pose Estimator");
    return estimatorMultiCamera;
  }

  private void configureGoToBranch() {
    this.goToBranchConfigurationFast = new GoToBranchConfiguration(
        AutoAlignConstants.PoseDeadBand.Fast.MIN_ERROR_AUTO_ALIGN_FAST,
        AutoAlignConstants.PoseDeadBand.Fast.MAX_ERROR_AUTO_ALIGN_FAST,
        AutoAlignConstants.PoseDeadBand.Fast.ERROR_FOR_ROTATION_ALIGN_ACTIVATION_FAST, "FAST",
        AutoAlignConstants.VelocitiesRelatedToDistance.Fast.MIN_VELOCITY_POSITION,
        AutoAlignConstants.VelocitiesRelatedToDistance.Fast.MAX_VELOCITY_POSITION);
    this.goToBranchConfigurationTeleoperated = new GoToBranchConfiguration(
        AutoAlignConstants.PoseDeadBand.Teleoperated.MIN_ERROR_AUTO_ALIGN_TELEOPERATED,
        AutoAlignConstants.PoseDeadBand.Teleoperated.MAX_ERROR_AUTO_ALIGN_TELEOPERATED,
        AutoAlignConstants.PoseDeadBand.Teleoperated.ERROR_FOR_ROTATION_ALIGN_ACTIVATION_TELEOPERATED, "TELEOPERATED",
        AutoAlignConstants.VelocitiesRelatedToDistance.Teleoperated.MIN_VELOCITY_POSITION,
        AutoAlignConstants.VelocitiesRelatedToDistance.Teleoperated.MAX_VELOCITY_POSITION);
  }

  public void resetOdometryLimelight(Translation2d defaultPosition) {
    PoseEstimator limelightLeft = new LimelightPoseEstimator("limelight-left", false, false, 2);
    Optional<PoseEstimation> limelightPoseEstimation = limelightLeft.getEstimatedPose(this.getPose());
    if (limelightPoseEstimation.isEmpty()) {
      resetTranslation(defaultPosition);
      this.positionUpdated = true;
    } else {
      resetTranslation(limelightPoseEstimation.get().estimatedPose.getTranslation().toTranslation2d());
      this.positionUpdated = true;
    }
  }

  public boolean positionUpdated() {
    return this.positionUpdated;
  }

  @Override
  public void driveAlignAngleJoystick() {
    forceReefPoseEstimation(false);
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
        controller.getXtranslation(), rotateRight ? -1.5 : 1.5);
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
    SmartDashboard.putBoolean("BACKUP NECESSARY", checkBackupNecessary());
    selectPoseEstimator();
    super.periodic();
    updateLogs();
    LimelightHelpers.SetRobotOrientation("limelight-left",
        OdometryEnabledSwerveSubsystem.robotOrientation,
        OdometryEnabledSwerveSubsystem.robotAngularVelocity, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-right",
        OdometryEnabledSwerveSubsystem.robotOrientation,
        OdometryEnabledSwerveSubsystem.robotAngularVelocity, 0, 0, 0, 0);
  }

  private void selectPoseEstimator() {
    forcingReefPoseEstimatorLogger.append(forceReefPoseEstimation);
    if (getPose().getTranslation()
        .getDistance(AllianceFlipUtil
            .apply(FieldConstants.Reef.center)) < SwerveConstants.DISTANCE_FROM_REEF_CENTER_TO_USE_REEF_POSE_ESTIMATION
        && (state.contains("DRIVE_TO_BRANCH") || state.contains("STOP") || this.forceReefPoseEstimation)) {
      poseEstimatorState = PoseEstimatorState.REEF_ESTIMATION;
    } else {
      poseEstimatorState = PoseEstimatorState.GLOBAL_POSE_ESTIMATION;
    }
    switch (poseEstimatorState) {
      case GLOBAL_POSE_ESTIMATION:
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight-left", this.apriltagsIDs);
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight-right", this.apriltagsIDs);
        overrideTeleOpPoseEstimator(null);
        overrideAutonomousPoseEstimator(null);
        break;
      case REEF_ESTIMATION:
        overrideTeleOpPoseEstimator(reefPoseEstimator);
        overrideAutonomousPoseEstimator(reefPoseEstimator);
        break;
      default:
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight-left", this.apriltagsIDs);
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight-right", this.apriltagsIDs);
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
    this.targetBranchLogger.append(this.targetBranch.name());
  }

  @Override
  public void driveToBranch(TargetBranch branch, boolean backup, boolean goDirect) {
    this.targetBranch = branch;
    if (DriverStation.isAutonomousEnabled()) {
      driveToBranchFast(branch, backup, goDirect);
      return;
    }
    goToBranchTeleoperated(branch, backup, goDirect);
  }

  private void driveToBranchFast(TargetBranch branch, boolean backup, boolean goDirect) {
    this.goToBranchConfigurationFast.setBranch(branch, goDirect);
    this.goToBranchConfigurationFast.updateBranchData(getPose(), scorerTargetReefLevelSupplier,
        scorerTargetReefLevelAlgaeSupplier,
        elevatorAtHighPositionSupplier, backup, false);
    this.distanceToTargetBranch = goToBranchConfigurationFast.getDistanceToTargetBranch();
    this.targetVelocity.append(goToBranchConfigurationFast.getFinalVelocity());
    this.distanceToTargetBranchLog.append(distanceToTargetBranch);
    this.isUsingAngleCorrection.append(false);
    driveToPose(this.goToBranchConfigurationFast.getFinalPose(),
        this.goToBranchConfigurationFast.getFinalVelocity());
    this.state = this.goToBranchConfigurationFast.getGoToState();
  }

  private void goToBranchTeleoperated(TargetBranch branch, boolean backup, boolean goDirect) {
    this.goToBranchConfigurationTeleoperated.setBranch(branch, goDirect);
    this.goToBranchConfigurationTeleoperated.updateBranchData(getPose(), scorerTargetReefLevelSupplier,
        scorerTargetReefLevelAlgaeSupplier,
        elevatorAtHighPositionSupplier, backup, false);
    this.distanceToTargetBranch = goToBranchConfigurationTeleoperated.getDistanceToTargetBranch();
    this.targetVelocity.append(goToBranchConfigurationTeleoperated.getFinalVelocity());
    this.distanceToTargetBranchLog.append(distanceToTargetBranch);
    if (this.goToBranchConfigurationTeleoperated.getDistanceToTargetBranch() < 3.5) {
      this.isUsingAngleCorrection.append(false);
      driveToPose(this.goToBranchConfigurationTeleoperated.getFinalPose(),
          this.goToBranchConfigurationTeleoperated.getFinalVelocity());
      this.state = this.goToBranchConfigurationTeleoperated.getGoToState();
    } else {
      driveAlignAngleJoystick();
    }
  }

  @Override
  public void goToFaceTeleoperated(TargetBranch branch) {
    this.goToBranchConfigurationTeleoperated.setBranch(branch, true);
    this.goToBranchConfigurationTeleoperated.updateBranchData(getPose(), scorerTargetReefLevelSupplier,
        scorerTargetReefLevelAlgaeSupplier,
        elevatorAtHighPositionSupplier, false, true);
    this.distanceToTargetFace = goToBranchConfigurationTeleoperated.getDistanceToTargetFace();
    this.targetVelocity.append(goToBranchConfigurationTeleoperated.getFinalVelocity());
    this.distanceToTargetFaceLog.append(distanceToTargetFace);
    this.isUsingAngleCorrection.append(false);
    driveToPose(this.goToBranchConfigurationTeleoperated.getFinalPose(),
        this.goToBranchConfigurationTeleoperated.getFinalVelocity());
    this.targetFacePose = this.goToBranchConfigurationTeleoperated.getFinalPose();
    this.state = this.goToBranchConfigurationTeleoperated.getGoToState();
  }

  @Override
  public boolean isAtTargetFacePositionWithoutHeading() {
    return stableAtTargetPose
        .isStableInCondition(() -> isAtTargetPose(this.targetFacePose, this.goToPoseTranslationDeadband,
            this.goToPoseTranslationDeadband));
  }

  @Override
  public boolean checkBackupNecessary() {
    return getPose().getTranslation().getDistance(targetBranch.getTargetPoseToScore().getTranslation()) < 1;
  }

  @Override
  public boolean checkPivotWillCrashOnReef() {
    return getPose().getTranslation().getDistance(targetBranch.getTargetPoseToScore().getTranslation()) < 0.2;
  }

  @Override
  public double getDistanceToTargetBranch() {
    return this.distanceToTargetBranch;
  }

  private TargetBranch getTargetBranch() {
    return this.targetBranch;
  }

  private Pose2d getNearestCoralStationPose() {
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      if (this.getPose().getY() >= 4.0259) {
        return SwerveConstants.CoralStations.RedAliance.CORAL_STATION_RIGHT_POSE_FOR_ROBOT;
      } else {
        return SwerveConstants.CoralStations.RedAliance.CORAL_STATION_LEFT_POSE_FOR_ROBOT;
      }
    } else {
      if (this.getPose().getY() >= 4.0259) {
        return SwerveConstants.CoralStations.BlueAliance.CORAL_STATION_LEFT_POSE_FOR_ROBOT;
      } else {
        return SwerveConstants.CoralStations.BlueAliance.CORAL_STATION_RIGHT_POSE_FOR_ROBOT;
      }
    }
  }

  public void setAngleForClimb() {
    if (this.getPose().getX() >= FieldConstants.fieldLength / 2) {
      this.bestAngleForClimb = Rotation2d.fromDegrees(90);
    } else {
      this.bestAngleForClimb = Rotation2d.fromDegrees(-90);
    }
  }

  @Override
  public void driveLockedAngleToNearestCoralStation() {
    if (controller.leftBumper().getAsBoolean()) {
      driveRotating(false);
      return;
    }
    if (controller.rightBumper().getAsBoolean()) {
      driveRotating(true);
      return;
    }
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
  public boolean isAtTargetPositionWithHeading() {
    return stableAtTargetPose.isStableInCondition(() -> isAtTargetPose(this.goToPoseTranslationDeadband,
        this.goToPoseTranslationDeadband, this.goToPoseHeadingDeadband));
  }

  @Override
  public boolean isAtTargetPositionWithoutHeading() {
    return stableAtTargetPose.isStableInCondition(() -> isAtTargetPose(this.goToPoseTranslationDeadband,
        this.goToPoseTranslationDeadband));
  }

  public void driveToPoseTest() {
    driveToPose(new Pose2d(15, 2, new Rotation2d(Units.degreesToRadians(180))));
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
        controller.getXtranslation()).times(0.5);
    this.state = "DRIVE_ALIGN_ANGLE_JOY_REMOVE_ALGAE";
    this.driveFieldOrientedLockedJoystickAngle(desiredSpeeds, controller.getCOS_Joystick(),
        controller.getSIN_Joystick());
  }

  @Override
  public boolean swerveIsToCloseToReefForLiftingElevador() {
    return getPose().getTranslation().getDistance(AllianceFlipUtil.apply(Reef.center)) < 1;
  }

}
