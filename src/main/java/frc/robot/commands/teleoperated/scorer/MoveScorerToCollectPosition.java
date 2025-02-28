package frc.robot.commands.teleoperated.scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructure;

public class MoveScorerToCollectPosition extends Command {
  SuperStructure superStructure;

  public MoveScorerToCollectPosition(SuperStructure superStructure) {
    this.superStructure = superStructure;
    addRequirements(this.superStructure);
  }

  @Override
  public void initialize() {
    superStructure.intake.stopIntake();
    superStructure.scorer.intakeFromHP();
  }

  @Override
  public void execute() {
    superStructure.scorer.intakeFromHP();
  }

  @Override
  public boolean isFinished() {
    return superStructure.scorer.isAtCollectPosition();
  }
}
