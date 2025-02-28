package frc.robot.commands.teleoperated.states;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.teleoperated.scorer.MoveScorerToScorePosition;
import frc.robot.commands.teleoperated.swerve.SwerveGoToBranch;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoScoreCoralPosition extends SequentialCommandGroup {

  public AutoScoreCoralPosition(SuperStructure superStructure, SwerveSubsystem swerve, TargetBranch branch) {
    addCommands(new SwerveGoToBranch(swerve, branch, true), new MoveScorerToScorePosition(superStructure),
        new SwerveGoToBranch(swerve, branch, false),
        Commands.run(() -> superStructure.scorer.placeCoral(), superStructure).withTimeout(Seconds.of(1)),
        Commands.idle(superStructure));
  }
}
