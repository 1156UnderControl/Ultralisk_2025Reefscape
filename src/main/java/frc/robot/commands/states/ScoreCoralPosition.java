package frc.robot.commands.states;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.scorer.MoveScorerToScorePosition;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ScoreCoralPosition extends SequentialCommandGroup {
  OperatorController operatorController = OperatorController.getInstance();

  public ScoreCoralPosition(SuperStructure superStructure, SwerveSubsystem swerve) {
    addCommands(new MoveScorerToScorePosition(superStructure),
        Commands.waitUntil(operatorController.scoreCoral()),
        Commands.run(() -> superStructure.scorer.placeCoral(), superStructure).withTimeout(Seconds.of(1)),
        Commands.idle(superStructure));
  }
}
