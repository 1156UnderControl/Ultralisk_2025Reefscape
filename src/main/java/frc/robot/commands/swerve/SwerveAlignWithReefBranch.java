package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.ISwerve;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SwerveAlignWithReefBranch extends Command {
  ISwerve swerve;
  TargetBranch targetBranch;

  public SwerveAlignWithReefBranch(SwerveSubsystem swerve, TargetBranch branch) {
    this.swerve = swerve;
    this.targetBranch = branch;
    addRequirements(swerve);
  }

  @Override
  public void execute() {
    this.swerve.driveToBranch(this.targetBranch);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
