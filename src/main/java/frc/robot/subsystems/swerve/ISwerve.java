package frc.robot.subsystems.swerve;

import frc.robot.constants.SwerveConstants.TargetBranch;

public interface ISwerve {
  public void driveLockedAngleToNearestCoralStation();

  public void driveLockedAngleToClimb();

  public void driveToNearestCoralStation();

  public void driveToBranch(TargetBranch branch);

  public void driveToBranchFastDirect(TargetBranch branch);

  public void driveToBranchFast(TargetBranch branch);

  public void driveAlignAngleJoystick();

  public void driveAlignAngleJoystickSuperSlow();

  public void stopSwerve();

  public double getDistanceToTargetBranch();

  public boolean swerveIsToCloseToReefForLiftingElevador();

  public boolean isAtTargetPositionWithHeading();

  public boolean isAtTargetPositionWithoutHeading();
}
