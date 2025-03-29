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

  public SwerveGoToBranchFastAutonomous(SwerveSubsystem swerve, TargetBranch branch, boolean isSpacedToBranch,
      boolean goDirect) {
    this.swerve = swerve;
    this.targetBranch = branch;
    this.isSpacedToBranch = isSpacedToBranch;
    this.goDirect = goDirect;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (goDirect) {
      this.swerve.driveToBranchFastDirect(targetBranch, isSpacedToBranch);
    } else {
      this.swerve.driveToBranchFast(targetBranch, isSpacedToBranch);
    }
  }

  @Override
  public boolean isFinished() {
    return this.swerve.isAtTargetPositionWithoutHeading();
  }

  @Override
  public void end(boolean interrupted) {
    this.swerve.stopSwerve();
  }
}
