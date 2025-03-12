package frc.robot.commands.scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructure;
import frc.robot.joysticks.OperatorController;

public class OptimizedMoveScorerToScorePosition extends Command {
  SuperStructure superStructure;
  OperatorController operatorController = OperatorController.getInstance();

  public OptimizedMoveScorerToScorePosition(SuperStructure superStructure, SwerveSubsystem swerve) {
    this.superStructure = superStructure;
    addRequirements(this.superStructure);
  }

  @Override
  public void initialize() {
    superStructure.scorer.prepareToPlaceCoralOnBranch(swerve.getDistanceToTargetBranch);
  }

  @Override
  public void execute() {
    superStructure.scorer.prepareToPlaceCoralOnBranch(swerve.getDistanceToTargetBranch);
  }

  @Override
  public boolean isFinished() {
    return superStructure.scorer.isSecuredToPlaceCoral();
  }
}
