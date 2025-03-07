package frc.robot.commands.autonomous_commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.intake.CollectCoralFromHP;
import frc.robot.commands.scorer.MoveScorerToScorePosition;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoScoreCoralAutonomousOptimized extends SequentialCommandGroup {

  public AutoScoreCoralAutonomousOptimized(SuperStructure superStructure, SwerveSubsystem swerve, TargetBranch branch) {
    addCommands(new InstantCommand(() -> swerve.forceReefPoseEstimation(true)),
        new SwerveGoToBranchFastAutonomous(swerve, branch, true)
            .alongWith(new CollectCoralFromHP(superStructure).withTimeout(1)
                .andThen(Commands.waitUntil(() -> swerve.getDistanceToTargetBranch() < 1.5)
                    .andThen(new MoveScorerToScorePosition(superStructure)))),
        new SwerveGoToBranchFastAutonomous(swerve, branch, false),
        Commands.run(() -> superStructure.scorer.placeCoral()).withTimeout(0.3),
        // new SwerveGoToBranchFastAutonomous(swerve, branch, true).withTimeout(0.3),
        new InstantCommand(() -> swerve.forceReefPoseEstimation(false)));
  }
}
