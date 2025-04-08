package frc.robot.commands.scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructure;

public class ScoreProcessor extends Command {
  SuperStructure superStructure;

  public ScoreProcessor(SuperStructure superStructure) {
    this.superStructure = superStructure;
    addRequirements(this.superStructure);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    superStructure.scorer.placeAlgae();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
