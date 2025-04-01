package frc.robot.commands.states;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.scorer.MoveScorerToScoreAlgaePosition;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ScoreAlgaeNetPosition extends SequentialCommandGroup {
  OperatorController operatorKeyboard = OperatorController.getInstance();

  public ScoreAlgaeNetPosition(SuperStructure superStructure, SwerveSubsystem swerve) {
    addCommands(new MoveScorerToScoreAlgaePosition(superStructure),
        Commands.waitUntil(operatorKeyboard.scoreAlgae()),
        Commands.run(() -> superStructure.scorer.placeCoral(), superStructure).withTimeout(Seconds.of(1)),
        Commands.idle(superStructure));
  }
}
