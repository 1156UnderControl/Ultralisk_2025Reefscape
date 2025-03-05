package frc.robot.commands.teleoperated.scorer;

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
    superStructure.scorer.removeAlgaeFromBranch();
  }

  @Override
  public void execute() {
    superStructure.scorer.removeAlgaeFromBranch();
    if (superStructure.scorer.isAtRemovePosition()) {
      superStructure.scorer.removeAlgaeEndEffector();
    } else {
      superStructure.scorer.stopEndEffector();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
