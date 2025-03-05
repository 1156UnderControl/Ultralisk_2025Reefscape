package frc.robot.commands.teleoperated.scorer;

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
    superStructure.intake.stopIntake();
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
