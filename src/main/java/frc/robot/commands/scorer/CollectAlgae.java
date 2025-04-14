package frc.robot.commands.scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructure;

public class CollectAlgae extends Command {
  SuperStructure superStructure;

  public CollectAlgae(SuperStructure superStructure) {
    this.superStructure = superStructure;
    addRequirements(this.superStructure);
  }

  @Override
  public void initialize() {
    superStructure.scorer.runEndEffectorForAlgaeRemoval();
    ;
  }

  @Override
  public void execute() {
    superStructure.scorer.runEndEffectorForAlgaeRemoval();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return superStructure.scorer.hasAlgae();
  }
}
