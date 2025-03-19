package frc.robot.commands.autonomous_commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.intake.CollectCoralFromHP;
import frc.robot.commands.scorer.MoveScorerToScorePosition;
import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoScoreCoralAutonomousOptimizedDirect extends SequentialCommandGroup {

  public AutoScoreCoralAutonomousOptimizedDirect(SuperStructure superStructure, SwerveSubsystem swerve,
      TargetBranch branch) {
    addCommands(new InstantCommand(() -> swerve.forceReefPoseEstimation(true)),
        new SwerveGoToBranchFastAutonomous(swerve, branch, true, true)
            .alongWith(new CollectCoralFromHP(superStructure)
                .andThen(
                    new InstantCommand(() -> superStructure.scorer.setTargetBranchLevel(ReefLevel.TO_L4),
                        superStructure).unless(() -> swerve.getDistanceToTargetBranch() < 1.5)
                        .andThen(new MoveScorerToScorePosition(superStructure))
                        .andThen(Commands.waitUntil(() -> swerve.getDistanceToTargetBranch() < 1.5)
                            .andThen(new InstantCommand(() -> superStructure.scorer.setTargetBranchLevel(ReefLevel.L4),
                                superStructure).andThen(new MoveScorerToScorePosition(superStructure)))))),
        new SwerveGoToBranchFastAutonomous(swerve, branch, false, false),
        Commands.run(() -> superStructure.scorer.placeCoral()).withTimeout(0.3),
        new InstantCommand(() -> swerve.forceReefPoseEstimation(false)));
  }
}
