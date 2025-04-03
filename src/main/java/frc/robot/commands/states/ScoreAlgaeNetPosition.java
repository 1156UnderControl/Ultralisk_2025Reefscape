package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ScoreAlgaeNetPosition extends SequentialCommandGroup {
  OperatorController operatorKeyboard = OperatorController.getInstance();

  public ScoreAlgaeNetPosition(SuperStructure superStructure, SwerveSubsystem swerve) {
    addCommands(
    // new ParallelCommandGroup(Commands.run(() ->
    // superStructure.scorer.holdAlgae(), superStructure),
    // new MoveScorerToRemovePosition(superStructure)),
    // Commands.waitUntil(operatorKeyboard.scoreAlgae()),
    // Commands.run(() -> superStructure.scorer.placeAlgae(),
    // superStructure).withTimeout(Seconds.of(1)),
    // Commands.idle(superStructure)
    );
  }
}
