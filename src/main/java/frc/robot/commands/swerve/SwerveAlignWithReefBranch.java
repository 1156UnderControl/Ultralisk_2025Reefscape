package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.ISwerve;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SwerveAlignWithReefBranch extends Command {
  ISwerve swerve;
  TargetBranch targetBranch;
  boolean backup;
  boolean goDirect;

  public SwerveAlignWithReefBranch(SwerveSubsystem swerve, TargetBranch branch, boolean backup, boolean goDirect) {
    this.swerve = swerve;
    this.targetBranch = branch;
    this.backup = backup;
    this.goDirect = goDirect;
    addRequirements(swerve);
  }

  @Override
  public void execute() {
    this.swerve.driveToBranch(this.targetBranch, this.backup, this.goDirect);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
