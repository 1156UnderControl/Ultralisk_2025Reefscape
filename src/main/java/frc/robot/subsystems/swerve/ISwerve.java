package frc.robot.subsystems.swerve;

import frc.robot.constants.SwerveConstants.TargetBranch;

public interface ISwerve {
  public void driveLockedAngleToNearestCoralStation();

  public void driveLockedAngleToClimb();

  public void driveToNearestCoralStation();

  public void driveToBranch(TargetBranch branch, boolean backup, boolean goDirect);

  public void driveToBranchFastDirect(TargetBranch branch, boolean backup, boolean goDirect);

  public void driveToBranchFast(TargetBranch branch, boolean backup, boolean goDirect);

  boolean checkBackupNecessary();

  boolean checkPivotWillCrashOnReef();

  public void driveAlignAngleJoystick();

  public void driveAlignAngleJoystickSuperSlow();

  public void stopSwerve();

  public double getDistanceToTargetBranch();

  public boolean swerveIsToCloseToReefForLiftingElevador();

  public boolean isAtTargetPositionWithHeading();

  public boolean isAtTargetPositionWithoutHeading();

  public void setTargetBranch(TargetBranch branch);
}
