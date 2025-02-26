package frc.robot.commands.states;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Java_Is_UnderControl.LEDs.LedColor;
import frc.Java_Is_UnderControl.LEDs.LedSubsystem;
import frc.robot.SuperStructure;
import frc.robot.commands.scorer.MoveScorerToScorePosition;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ScoreCoralPosition extends SequentialCommandGroup {

  public ScoreCoralPosition(SuperStructure superStructure, SwerveSubsystem swerve, LedSubsystem led) {
    addCommands(
        Commands.runOnce(() -> led.setSolidColor(LedColor.BLUE)),
        withTimeout(Seconds.of(1)),
        new MoveScorerToScorePosition(superStructure),
        beforeStarting(() -> led.setSolidColor(LedColor.RED)),
        withTimeout(Seconds.of(1)),
        Commands.run(() -> superStructure.scorer.placeCoral(), superStructure).withTimeout(Seconds.of(1)),
        Commands.runOnce(() -> led.setSolidColor(LedColor.GREEN)),
        withTimeout(Seconds.of(1)),
        Commands.idle(superStructure));
  }
}
