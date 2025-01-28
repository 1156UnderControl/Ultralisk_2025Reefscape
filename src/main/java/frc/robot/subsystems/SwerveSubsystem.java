package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.Java_Is_UnderControl.Control.PIDConfig;
import frc.Java_Is_UnderControl.Swerve.OdometryEnabledSwerveConfig;
import frc.Java_Is_UnderControl.Swerve.OdometryEnabledSwerveSubsystem;
import frc.Java_Is_UnderControl.Swerve.SwervePathPlannerConfig;
import frc.Java_Is_UnderControl.Vision.Odometry.NoPoseEstimator;
import frc.robot.Joysticks.ControlBoard;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class SwerveSubsystem extends OdometryEnabledSwerveSubsystem implements Subsystem {

  private ControlBoard controller = ControlBoard.getInstance();

  private Matrix<N3, N1> defaultVisionStandardDeviation;

  private Matrix<N3, N1> defaultOdometryStandardDeviation;

  private String state = "NULL";

  public SwerveSubsystem(
      SwerveDrivetrainConstants drivetrainConstants,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(new OdometryEnabledSwerveConfig(0.75, new SwervePathPlannerConfig(
        new PIDConstants(5, 0, 0),
        new PIDConstants(5, 0, 0)),
        new NoPoseEstimator(),
        new NoPoseEstimator(),
        new PIDConfig(6, 0, 0),
        new PIDConfig(0.5, 0, 0)),
        drivetrainConstants,
        modules);
  }

  public void driveAlignAngleJoy() {
    ChassisSpeeds desiredSpeeds = this.inputsToChassisSpeeds(-controller.getYtranslation(),
        -controller.getXtranslation());
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

  @Override
  public void periodic() {

  }

  @Override
  protected void updateLogs() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateLogs'");
  }
}
