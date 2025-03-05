package frc.robot.commands.scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructure;

public class MoveScorerToDefaultPositionWithCoral extends Command {
  SuperStructure superStructure;

  public MoveScorerToDefaultPositionWithCoral(SuperStructure superStructure) {
    this.superStructure = superStructure;
    addRequirements(this.superStructure);
  }

  @Override
  public void initialize() {
    superStructure.scorer.moveScorerToDefaultPosition();
  }

  @Override
  public void execute() {
    superStructure.scorer.moveScorerToDefaultPosition();
  }

  @Override
  public boolean isFinished() {
    return superStructure.scorer.isAtDefaultPosition();
  }
}
