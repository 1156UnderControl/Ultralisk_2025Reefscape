package frc.robot.commands.teleoperated.teleop_states;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
<<<<<<< HEAD:src/main/java/frc/robot/commands/teleoperated/teleop_states/ScoreCoralPosition.java
import frc.robot.commands.teleoperated.scorer.MoveScorerToScorePosition;
=======
import frc.robot.commands.scorer.MoveScorerToScorePosition;
import frc.robot.joysticks.OperatorController;
>>>>>>> main:src/main/java/frc/robot/commands/states/ScoreCoralPosition.java
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ScoreCoralPosition extends SequentialCommandGroup {
  OperatorController operatorKeyboard = OperatorController.getInstance();

  public ScoreCoralPosition(SuperStructure superStructure, SwerveSubsystem swerve) {
    addCommands(new MoveScorerToScorePosition(superStructure),
        Commands.waitUntil(operatorKeyboard.scoreCoral()),
        Commands.run(() -> superStructure.scorer.placeCoral(), superStructure).withTimeout(Seconds.of(1)),
        Commands.idle(superStructure));
  }
}
