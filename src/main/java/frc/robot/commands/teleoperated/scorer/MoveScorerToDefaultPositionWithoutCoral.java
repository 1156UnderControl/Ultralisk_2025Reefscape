package frc.robot.commands.scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructure;

public class MoveScorerToDefaultPositionWithoutCoral extends Command {
  SuperStructure superStructure;

  public MoveScorerToDefaultPositionWithoutCoral(SuperStructure superStructure) {
    this.superStructure = superStructure;
    addRequirements(this.superStructure);
  }

  @Override
  public void initialize() {
    superStructure.scorer.moveScorerToDefaultPositionWithoutCoral();
    ;
  }

  @Override
  public void execute() {
    superStructure.scorer.moveScorerToDefaultPositionWithoutCoral();
    ;
  }

  @Override
  public boolean isFinished() {
    return superStructure.scorer.isAtDefaultPosition();
  }
}
