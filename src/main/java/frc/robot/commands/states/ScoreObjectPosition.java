package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.scorer.MoveScorerToScorePosition;
import frc.robot.joysticks.OperatorController;

public class ScoreObjectPosition extends SequentialCommandGroup {
  OperatorController operatorKeyboard = OperatorController.getInstance();

  public ScoreObjectPosition(SuperStructure superStructure) {
    addCommands(new MoveScorerToScorePosition(superStructure).until(operatorKeyboard.scoreObject().and(() -> {
      if (superStructure.scorer.hasCoral()) {
        return superStructure.scorer.isSecuredToPlaceCoral();
      }
      if (superStructure.scorer.hasAlgae()) {
        return superStructure.scorer.isSecuredToScoreOnNet();
      }
      return superStructure.scorer.isSecuredToPlaceCoral();
    })),
        Commands.either(Commands.run(() -> superStructure.scorer.placeCoral(), superStructure),
            Commands.run(() -> superStructure.scorer.placeAlgae(), superStructure),
            () -> superStructure.scorer.hasCoral()).withTimeout(1),
        Commands.idle(superStructure));
  }
}
