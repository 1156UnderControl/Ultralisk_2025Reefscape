package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.scorer.MoveScorerToScorePosition;
import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class GoAndRaiseElevator extends Command {

  SwerveSubsystem swerve;
  TargetBranch targetBranch;
  SuperStructure superStructure;

  public GoAndRaiseElevator(SwerveSubsystem swerve, SuperStructure superStructure, TargetBranch branch) {
    this.swerve = swerve;
    this.superStructure = superStructure;
    this.targetBranch = branch;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (this.superStructure.scorer.getTargetReefLevel() != ReefLevel.L4) {
      new SequentialCommandGroup(new MoveScorerToScorePosition(superStructure),
          Commands.run(() -> this.swerve.driveToBranch(this.targetBranch, true)));
    } else {
      new SequentialCommandGroup(Commands.run(() -> this.swerve.driveToBranch(this.targetBranch, true)),
          new MoveScorerToScorePosition(superStructure));
    }
  }

  @Override
  public boolean isFinished() {
    return this.swerve.isAtTargetPosition() && this.superStructure.scorer.isSecuredToPlaceCoral();
  }

  @Override
  public void end(boolean interrupted) {
  }
}
