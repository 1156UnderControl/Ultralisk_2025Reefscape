package frc.robot.commands.autonomous_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.ISwerve;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SwerveGoToBranchFastAutonomous extends Command {
  ISwerve swerve;
  TargetBranch targetBranch;
  boolean isSpacedToBranch;

  public SwerveGoToBranchFastAutonomous(SwerveSubsystem swerve, TargetBranch branch, boolean isSpacedToBranch) {
    this.swerve = swerve;
    this.targetBranch = branch;
    this.isSpacedToBranch = isSpacedToBranch;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    this.swerve.driveToBranchFast(targetBranch, isSpacedToBranch);
  }

  @Override
  public boolean isFinished() {
    return this.swerve.isAtTargetPosition();
  }

  @Override
  public void end(boolean interrupted) {
    this.swerve.stopSwerve();
  }
}
