package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.ISwerve;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SwerveGoToBranch extends Command {
  ISwerve swerve;
  TargetBranch targetBranch;
  boolean isBackupNecessary;
  boolean reachedBackupPosition = false;
  boolean isGoingToNonBackupPosition;
  boolean goDirect;

  public SwerveGoToBranch(SwerveSubsystem swerve, TargetBranch branch) {
    this.swerve = swerve;
    this.targetBranch = branch;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    this.swerve.setTargetBranch(targetBranch);
    isBackupNecessary = this.swerve.checkBackupNecessary();
    isGoingToNonBackupPosition = false;
  }

  @Override
  public void execute() {
    if (isBackupNecessary && !reachedBackupPosition) {
      this.swerve.driveToBranch(targetBranch, true, true);
      if (this.swerve.isAtTargetPositionWithoutHeading()) {
        reachedBackupPosition = true;
      }
    } else {
      this.swerve.driveToBranch(targetBranch, false, false);
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
