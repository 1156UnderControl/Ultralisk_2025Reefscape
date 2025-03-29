package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Java_Is_UnderControl.LEDs.LedColor;
import frc.robot.SuperStructure;
import frc.robot.commands.intake.CollectCoralFromHP;
import frc.robot.commands.scorer.MoveScorerToCollectPosition;
import frc.robot.commands.swerve.SwerveAlignWithCoralStation;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoIntakeCoralPosition extends SequentialCommandGroup {

  public AutoIntakeCoralPosition(SuperStructure superStructure, SwerveSubsystem swerve) {
    addCommands(new MoveScorerToCollectPosition(superStructure),
        Commands.race(new CollectCoralFromHP(superStructure), new SwerveAlignWithCoralStation(swerve)),
        Commands.runOnce(() -> superStructure.led.setSolidColor(LedColor.WHITE)),
        Commands.waitSeconds(0.2));
  }
}
