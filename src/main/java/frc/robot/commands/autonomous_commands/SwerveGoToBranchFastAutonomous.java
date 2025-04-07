package frc.robot.commands.autonomous_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.ISwerve;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SwerveGoToBranchFastAutonomous extends Command {
  ISwerve swerve;
  TargetBranch targetBranch;
  boolean isSpacedToBranch;
  boolean goDirect;
  boolean isBackupNecessary;
  boolean reachedBackupPosition = false;
  boolean isGoingToNonBackupPosition;

  public SwerveGoToBranchFastAutonomous(SwerveSubsystem swerve, TargetBranch branch,
      boolean goDirect) {
    this.swerve = swerve;
    this.targetBranch = branch;
    this.goDirect = goDirect;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    isBackupNecessary = this.swerve.checkBackupNecessary();
    isGoingToNonBackupPosition = false;
  }

  @Override
  public void execute() {
    if (isBackupNecessary && !reachedBackupPosition) {
      if (goDirect) {
        this.swerve.driveToBranchFastDirect(targetBranch, true, true);
      } else {
        this.swerve.driveToBranchFast(targetBranch, true, false);
      }
      if (this.swerve.isAtTargetPositionWithoutHeading()) {
        reachedBackupPosition = true;
      }
    } else {
      if (goDirect) {
        this.swerve.driveToBranchFastDirect(targetBranch, false, true);
      } else {
        this.swerve.driveToBranchFast(targetBranch, false, false);
      }
      isGoingToNonBackupPosition = true;
    }
  }

  @Override
  public boolean isFinished() {
    return this.swerve.isAtTargetPositionWithoutHeading() && isGoingToNonBackupPosition;
  }

  @Override
  public void end(boolean interrupted) {
    this.swerve.stopSwerve();
  }
}
