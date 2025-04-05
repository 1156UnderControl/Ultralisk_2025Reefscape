package frc.robot.commands.states;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Java_Is_UnderControl.LEDs.LedColor;
import frc.robot.SuperStructure;
import frc.robot.commands.scorer.MoveScorerToScorePosition;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ScoreCoralPosition extends SequentialCommandGroup {
  OperatorController operatorKeyboard = OperatorController.getInstance();

  public ScoreCoralPosition(SuperStructure superStructure, SwerveSubsystem swerve) {
    addCommands(new MoveScorerToScorePosition(superStructure),
        new InstantCommand(() -> superStructure.led.setSolidColor(LedColor.RED)),
        Commands.waitUntil(operatorKeyboard.scoreCoral()),
        Commands.run(() -> superStructure.scorer.placeCoral(), superStructure).withTimeout(Seconds.of(1)),
        new InstantCommand(() -> superStructure.led.setSolidColor(LedColor.GREEN)),
        Commands.idle(superStructure));
  }
}
