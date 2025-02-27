package frc.robot.subsystems.swerve;

import frc.robot.constants.SwerveConstants.TargetBranch;

public interface ISwerve {
  public void driveAimingToNearestHP();

  public void driveToBranch(TargetBranch branch, boolean backupBranch);

  public boolean isAtTargetPosition();
}
