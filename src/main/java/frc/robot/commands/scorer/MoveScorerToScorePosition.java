package frc.robot.commands.scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructure;
import frc.robot.joysticks.OperatorController;

public class MoveScorerToScorePosition extends Command {
  SuperStructure superStructure;
  OperatorController operatorController = OperatorController.getInstance();

  public MoveScorerToScorePosition(SuperStructure superStructure) {
    this.superStructure = superStructure;
    addRequirements(this.superStructure);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (this.superStructure.scorer.hasCoral()) {
      superStructure.scorer.prepareToPlaceCoralOnBranch();
    }
    if (this.superStructure.scorer.hasAlgae()) {
      superStructure.scorer.prepareToScoreAlgae();
    }
  }

  @Override
  public boolean isFinished() {
    return superStructure.scorer.isSecuredToPlaceCoral();
  }
}
