package frc.robot.commands.scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructure;

public class MoveScorerToRemovePosition extends Command {
  SuperStructure superStructure;

  public MoveScorerToRemovePosition(SuperStructure superStructure) {
    this.superStructure = superStructure;
    addRequirements(this.superStructure);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    superStructure.scorer.collectAlgae();
  }

  @Override
  public void end(boolean interrupted) {
    superStructure.scorer.collectAlgae();
  }

  @Override
  public boolean isFinished() {
    return superStructure.scorer.hasAlgae();
  }
}
