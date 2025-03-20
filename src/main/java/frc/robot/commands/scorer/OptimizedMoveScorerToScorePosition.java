package frc.robot.commands.scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructure;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.swerve.ISwerve;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class OptimizedMoveScorerToScorePosition extends Command {
  SuperStructure superStructure;
  ISwerve drivetrain;
  OperatorController operatorController = OperatorController.getInstance();

  public OptimizedMoveScorerToScorePosition(SuperStructure superStructure, SwerveSubsystem drivetrain) {
    this.drivetrain = drivetrain;
    this.superStructure = superStructure;
    addRequirements(this.superStructure);
  }

  @Override
  public void initialize() {
    superStructure.scorer.prepareToPlaceCoralOnBranch(() -> drivetrain.getDistanceToTargetBranch());
  }

  @Override
  public void execute() {
    superStructure.scorer.prepareToPlaceCoralOnBranch(() -> drivetrain.getDistanceToTargetBranch());
  }

  @Override
  public boolean isFinished() {
    return superStructure.scorer.isSecuredToPlaceCoral();
  }
}
