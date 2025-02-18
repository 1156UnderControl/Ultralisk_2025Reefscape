package frc.robot.commands.scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructure;

public class MoveScorerToScorePosition extends Command {
  SuperStructure superStructure;

  public MoveScorerToScorePosition(SuperStructure superStructure) {
    this.superStructure = superStructure;
  }

  @Override
  public void initialize() {
    superStructure.scorer.prepareToPlaceCoralOnBranch();
  }

  @Override
  public void execute() {
    superStructure.scorer.prepareToPlaceCoralOnBranch();
  }

  @Override
  public boolean isFinished() {
    return superStructure.scorer.isSecuredToPlaceCoral();
  }
}
