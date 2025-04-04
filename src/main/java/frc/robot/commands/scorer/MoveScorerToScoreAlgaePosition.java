package frc.robot.commands.scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructure;
import frc.robot.joysticks.OperatorController;

public class MoveScorerToScoreAlgaePosition extends Command {
  SuperStructure superStructure;
  OperatorController operatorController = OperatorController.getInstance();

  public MoveScorerToScoreAlgaePosition(SuperStructure superStructure) {
    this.superStructure = superStructure;
    addRequirements(this.superStructure);
  }

  @Override
  public void initialize() {
    superStructure.scorer.prepareToScoreAlgae();
  }

  @Override
  public void execute() {
    superStructure.scorer.prepareToScoreAlgae();
  }

  @Override
  public boolean isFinished() {
    return superStructure.scorer.isSecuredToPlaceCoral();
  }
}
