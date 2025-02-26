package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Java_Is_UnderControl.LEDs.LedColor;
import frc.Java_Is_UnderControl.LEDs.LedSubsystem;
import frc.robot.SuperStructure;
import frc.robot.commands.intake.CollectCoralFromHP;
import frc.robot.commands.scorer.MoveScorerToCollectPosition;
import frc.robot.commands.swerve.SwerveAlignWithCoralStation;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class CollectPosition extends SequentialCommandGroup {

  public CollectPosition(SuperStructure superStructure, SwerveSubsystem swerve, LedSubsystem led) {
    addCommands(
        new MoveScorerToCollectPosition(superStructure),
        Commands.runOnce(() -> led.setSolidColor(LedColor.RED)),
        Commands.race(new CollectCoralFromHP(superStructure), new SwerveAlignWithCoralStation(swerve)),
        Commands.runOnce(() -> led.setSolidColor(LedColor.GREEN)),
        Commands.waitSeconds(0.2));

  }
}
