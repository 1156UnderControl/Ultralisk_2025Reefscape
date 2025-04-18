package frc.robot.commands.autonomous_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructure;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.ISwerve;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoGoToPoseCollectAlgaeFromReef extends Command {
  ISwerve swerve;
  TargetBranch targetBranch;
  SuperStructure superStructure;

  public AutoGoToPoseCollectAlgaeFromReef(SwerveSubsystem swerve, SuperStructure superStructure, TargetBranch branch) {
    this.swerve = swerve;
    this.targetBranch = branch;
    this.superStructure = superStructure;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    this.swerve.setTargetBranch(targetBranch);
  }

  @Override
  public void execute() {
    if (superStructure.scorer.isAtCollectAlgaePosition()) {
      this.swerve.goToCollectAlgaeFromFacePosition(targetBranch);
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
