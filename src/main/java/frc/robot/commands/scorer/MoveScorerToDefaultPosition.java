package frc.robot.commands.scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructure;

public class MoveScorerToDefaultPosition extends Command {
  SuperStructure superStructure;

  public MoveScorerToDefaultPosition(SuperStructure superStructure) {
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
