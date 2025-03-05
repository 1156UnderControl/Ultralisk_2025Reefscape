package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.teleoperated.scorer.MoveScorerToScorePosition;
import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class GoAndRaiseElevator extends SequentialCommandGroup {

  SwerveSubsystem swerve;
  TargetBranch targetBranch;
  SuperStructure superStructure;

  public GoAndRaiseElevator(SwerveSubsystem swerve, SuperStructure superStructure, TargetBranch branch) {
    this.swerve = swerve;
    this.superStructure = superStructure;
    this.targetBranch = branch;
    addCommands(Commands.either(
        new frc.robot.commands.teleoperated.scorer.MoveScorerToScorePosition(superStructure)
            .alongWith(new frc.robot.commands.teleoperated.swerve.SwerveGoToBranch(swerve, branch, true)),
        new frc.robot.commands.teleoperated.swerve.SwerveGoToBranch(swerve, branch, true)
            .andThen(new MoveScorerToScorePosition(superStructure)),
        () -> this.superStructure.scorer.getTargetReefLevel() != ReefLevel.L4
            && !swerve.swerveIsToCloseToReefForLiftingElevador()));
  }
}
