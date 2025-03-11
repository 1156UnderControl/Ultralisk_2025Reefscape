package frc.robot.commands.autonomous_commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoScoreCoralAutonomousOptimized extends SequentialCommandGroup {

  public AutoScoreCoralAutonomousOptimized(SuperStructure superStructure, SwerveSubsystem swerve, TargetBranch branch) {
    addCommands(new InstantCommand(() -> swerve.forceReefPoseEstimation(true)),
        new SwerveGoToBranchFastAutonomous(swerve, branch, true)
            .alongWith(new ConditionalCommand(new AutoScoreElevator(superStructure, swerve, branch), Commands.none(),
                () -> superStructure.scorer.hasCoral())));
  }
}
