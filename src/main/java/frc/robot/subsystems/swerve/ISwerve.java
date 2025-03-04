package frc.robot.subsystems.swerve;

import frc.robot.constants.SwerveConstants.TargetBranch;

public interface ISwerve {
  public void driveLockedAngleToNearestCoralStation();

  public void driveToNearestCoralStation();

  public void driveToBranch(TargetBranch branch, boolean backupBranch);

  public void driveAlignAngleJoystick();

  public void driveAlignAngleJoystickSuperSlow();

  public void stopSwerve();

  public boolean isAtTargetPosition();
}
