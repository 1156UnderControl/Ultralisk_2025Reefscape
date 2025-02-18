package frc.robot.commands.states;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.scorer.MoveScorerToCollectPosition;
import frc.robot.joysticks.ControlBoard;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ScoreCoralPosition extends SequentialCommandGroup {
  ControlBoard controlBoard = ControlBoard.getInstance();

  public ScoreCoralPosition(SuperStructure superStructure, SwerveSubsystem swerve) {
    addCommands(new MoveScorerToCollectPosition(superStructure),
        Commands.waitUntil(controlBoard.isForcingDriverControl()),
        Commands.run(() -> superStructure.scorer.placeCoral(), superStructure).withTimeout(Seconds.of(1)),
        Commands.idle(superStructure));
  }
}
