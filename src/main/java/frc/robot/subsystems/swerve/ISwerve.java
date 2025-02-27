package frc.robot.subsystems.swerve;

import frc.robot.constants.SwerveConstants.TargetBranch;

public interface ISwerve {
  public void driveLockedAngleToNearestCoralStation();

  public void driveToNearestCoralStation();

  public void driveToBranch(TargetBranch branch, boolean backupBranch);

  public boolean isAtTargetPosition();
}
