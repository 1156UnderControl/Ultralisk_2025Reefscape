package frc.robot.commands.states;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.scorer.MoveScorerToScorePosition;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ScoreObjectPosition extends SequentialCommandGroup {
  OperatorController operatorKeyboard = OperatorController.getInstance();

  public ScoreObjectPosition(SuperStructure superStructure, SwerveSubsystem swerve) {
    addCommands(new MoveScorerToScorePosition(superStructure),
        Commands.waitUntil(operatorKeyboard.scoreObject()),
        Commands.either(Commands.run(() -> superStructure.scorer.placeCoral(), superStructure),
            Commands.run(() -> superStructure.scorer.placeAlgae(), superStructure),
            () -> superStructure.scorer.hasCoral()).withTimeout(Seconds.of(1)),
        Commands.idle(superStructure));
  }
}
