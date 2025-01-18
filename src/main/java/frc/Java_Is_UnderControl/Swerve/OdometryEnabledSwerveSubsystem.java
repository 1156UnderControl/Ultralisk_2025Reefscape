package frc.Java_Is_UnderControl.Swerve;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomBooleanLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomPose2dLogger;
import frc.Java_Is_UnderControl.Vision.Odometry.PoseEstimation;
import frc.Java_Is_UnderControl.Vision.Odometry.PoseEstimator;

public abstract class OdometryEnabledSwerveSubsystem extends BaseSwerveSubsystem {

  public static double robotOrientation;

  public static double robotAngularVelocity;

  private PoseEstimator autonomousPoseEstimator;

  private PoseEstimator teleopPoseEstimator;

  private PIDController moveToPoseXAxisPid;

  private PIDController moveToPoseYAxisPid;

  private Pose2d targetPose;

  private Pose2d targetAimPose;

  private Pose2d poseVision;

  private CustomPose2dLogger targetPoseLogger = new CustomPose2dLogger("/SwerveSubsystem/TargetPose");

  private CustomPose2dLogger targetAimPositionLogger = new CustomPose2dLogger("/SwerveSubsystem/TargetAimPosition");

  private CustomBooleanLogger isAtTargetXAxisLogger = new CustomBooleanLogger("/SwerveSubsystem/IsAtTargetXAxis");

  private CustomBooleanLogger isAtTargetYAxisLogger = new CustomBooleanLogger("/SwerveSubsystem/IsAtTargetYAxis");

  private CustomBooleanLogger isAtTargetPoseLogger = new CustomBooleanLogger("/SwerveSubsystem/IsAtTargetPose");

  private CustomPose2dLogger poseVisionLogger = new CustomPose2dLogger("/SwerveSubsystem/PoseVision");

  // Load the path we want to pathfind to and follow
  // PathPlannerPath path = PathPlannerPath.fromPathFile("Go"); descobrir como nao
  // dar erro essa
  // Create the constraints to use while pathfinding. The constraints defined in
  // the path will only be used for the path.
  PathConstraints constraints = new PathConstraints(
      3.0, 4.0,
      Units.degreesToRadians(540), Units.degreesToRadians(720));

  // Since AutoBuilder is configured, we can use it to build pathfinding commands
  Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
      new PathPlannerPath(null, constraints, null, null),
      constraints);

  public OdometryEnabledSwerveSubsystem(OdometryEnabledSwerveConfig config,
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants... modules) {
    super(config, drivetrainConstants, modules);
    this.moveToPoseXAxisPid = new PIDController(config.moveToPosePIDConfig.kP, config.moveToPosePIDConfig.kI,
        config.moveToPosePIDConfig.kD);
    this.moveToPoseYAxisPid = new PIDController(config.moveToPosePIDConfig.kP, config.moveToPosePIDConfig.kI,
        config.moveToPosePIDConfig.kD);
    this.autonomousPoseEstimator = config.autonomousPoseEstimator;
    this.teleopPoseEstimator = config.teleoperatedPoseEstimator;
    this.targetPose = new Pose2d();
    this.targetAimPose = new Pose2d();
    this.poseVision = new Pose2d();
  }

  public OdometryEnabledSwerveSubsystem(OdometryEnabledSwerveConfig config,
      SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants... modules) {
    super(config, drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
        modules);
    this.moveToPoseXAxisPid = new PIDController(config.moveToPosePIDConfig.kP, config.moveToPosePIDConfig.kI,
        config.moveToPosePIDConfig.kD);
    this.moveToPoseYAxisPid = new PIDController(config.moveToPosePIDConfig.kP, config.moveToPosePIDConfig.kI,
        config.moveToPosePIDConfig.kD);
    this.autonomousPoseEstimator = config.autonomousPoseEstimator;
    this.teleopPoseEstimator = config.teleoperatedPoseEstimator;
    this.targetPose = new Pose2d();
    this.targetAimPose = new Pose2d();
    this.poseVision = new Pose2d();
  }

  private void updateOdometry() {
    if (DriverStation.isAutonomousEnabled() || DriverStation.isAutonomous()) {
      this.updateOdometryWithPoseEstimator(this.autonomousPoseEstimator);
    } else {
      this.updateOdometryWithPoseEstimator(this.teleopPoseEstimator);
    }
  }

  private void updateOdometryWithPoseEstimator(PoseEstimator poseEstimator) {
    Pose2d referencePose = getPose();
    Optional<PoseEstimation> possibleEstimatedPose = poseEstimator.getEstimatedPose(referencePose);
    if (possibleEstimatedPose.isPresent()) {
      PoseEstimation estimatedPose = possibleEstimatedPose.get();
      this.poseVision = estimatedPose.estimatedPose.toPose2d();
      super.addVisionMeasurement(this.poseVision, estimatedPose.timestampSeconds);
    }
  }

  protected double getDistanceToPosition(Translation2d targetPosition) {
    return this.getPose().getTranslation().getDistance(targetPosition);
  }

  protected void driveAimingAtPosition(ChassisSpeeds targetSpeeds, Translation2d targetPosition) {
    this.driveAimingAtPosition(targetSpeeds, targetPosition, Rotation2d.fromDegrees(0));
  }

  protected void driveAimingAtPosition(ChassisSpeeds targetSpeeds, Translation2d targetPosition,
      Rotation2d offsetAngle) {
    this.targetAimPose = new Pose2d(targetPosition, new Rotation2d());
    Translation2d positionDifference = this.getPose().getTranslation().minus(targetPosition);
    Rotation2d targetAngle = positionDifference.getAngle().plus(offsetAngle);
    this.driveFieldOrientedLockedAngle(targetSpeeds, targetAngle);
  }

  protected void driveAimingAtPositionMoving(double dt, ChassisSpeeds targetSpeeds, Translation2d targetPosition,
      Rotation2d offsetAngle) {
    this.targetAimPose = new Pose2d(targetPosition, new Rotation2d());
    Translation2d positionDifference = this.getEarlyPoseMoving(dt).getTranslation().minus(targetPosition);
    Rotation2d targetAngle = positionDifference.getAngle().plus(offsetAngle);
    this.driveFieldOrientedLockedAngle(targetSpeeds, targetAngle);
  }

  protected Rotation2d getHeadingAimingAtPositionMoving(double dt, Translation2d targetPosition,
      Rotation2d offsetAngle) {
    this.targetAimPose = new Pose2d(targetPosition, new Rotation2d());
    Translation2d positionDifference = this.getEarlyPoseMoving(dt).getTranslation().minus(targetPosition);
    Rotation2d targetAngle = positionDifference.getAngle().plus(offsetAngle);
    return targetAngle;
  }

  protected void driveAimingAtPositionMoving(double dt, ChassisSpeeds targetSpeeds, Translation2d targetPosition) {
    this.driveAimingAtPositionMoving(dt, targetSpeeds, targetPosition, Rotation2d.fromDegrees(0));
  }

  protected void driveToPose(Pose2d targetPose) {
    Pose2d currentPose = this.getPose();
    double targetXVelocity = this.moveToPoseXAxisPid.calculate(currentPose.getX(), targetPose.getX());
    double targetYVelocity = this.moveToPoseYAxisPid.calculate(currentPose.getY(), targetPose.getY());
    ChassisSpeeds desiredSpeeds = new ChassisSpeeds(targetXVelocity, targetYVelocity, 0);
    this.driveFieldOrientedLockedAngle(desiredSpeeds, targetPose.getRotation());
    this.targetPose = targetPose;
  }

  protected boolean isAtTargetPose(double xAxisToleranceMeters, double yAxisToleranceMeters,
      double angleToleranceDegrees) {
    double xAxisDistance = Math.abs(this.getPose().getX() - this.targetPose.getX());
    double yAxisDistance = Math.abs(this.getPose().getY() - this.targetPose.getY());
    boolean isAtXAxis = xAxisDistance <= xAxisToleranceMeters;
    boolean isAtYAxis = yAxisDistance <= yAxisToleranceMeters;
    boolean isAtAngle = this.isAtTargetHeading(angleToleranceDegrees);
    boolean isAtTargetPose = isAtXAxis && isAtYAxis && isAtAngle;
    this.isAtTargetXAxisLogger.append(isAtXAxis);
    this.isAtTargetYAxisLogger.append(isAtYAxis);
    this.isAtTargetPoseLogger.append(isAtTargetPose);
    return isAtTargetPose;
  }

  @Override
  protected void driveFieldOriented(ChassisSpeeds targetSpeeds) {
    this.targetPose = new Pose2d();
    super.driveFieldOriented(targetSpeeds);
  }

  @Override
  protected void driveRobotOriented(ChassisSpeeds targetSpeeds) {
    this.targetPose = new Pose2d();
    super.driveRobotOriented(targetSpeeds);
  }

  @Override
  public void periodic() {
    this.updateLimelightRequiredValues();
    this.updateOdometry();
    super.periodic();
    this.updateOdometrySwerveLogs();
  }

  private void updateLimelightRequiredValues() {
    OdometryEnabledSwerveSubsystem.robotAngularVelocity = this.getRobotVelocity().omegaRadiansPerSecond;
    OdometryEnabledSwerveSubsystem.robotOrientation = this.getState().Pose.getRotation().getDegrees();
  }

  private void updateOdometrySwerveLogs() {
    this.targetAimPositionLogger.appendRadians(targetAimPose);
    this.poseVisionLogger.appendRadians(this.poseVision);
    this.targetPoseLogger.appendRadians(this.targetPose);
  }

}
