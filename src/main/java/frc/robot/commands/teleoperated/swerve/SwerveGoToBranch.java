package frc.robot.commands.teleoperated.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.ISwerve;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SwerveGoToBranch extends Command {
  ISwerve swerve;
  TargetBranch targetBranch;
  boolean isSpacedToBranch;

  public SwerveGoToBranch(SwerveSubsystem swerve, TargetBranch branch, boolean isSpacedToBranch) {
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
    this.swerve.driveToBranch(this.targetBranch, this.isSpacedToBranch);
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
