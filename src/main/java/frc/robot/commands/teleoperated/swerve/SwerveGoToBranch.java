package frc.robot.commands.teleoperated.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SwerveGoToBranch extends Command {
  SwerveSubsystem swerve;
  TargetBranch targetBranch;

  public SwerveGoToBranch(SwerveSubsystem swerve, TargetBranch branch, boolean backupBranch) {
    this.swerve = swerve;
    this.targetBranch = branch;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    this.swerve.driveToBranch(this.targetBranch, false);
  }

  @Override
  public boolean isFinished() {
    return this.swerve.isAtTargetPosition();
  }
}
