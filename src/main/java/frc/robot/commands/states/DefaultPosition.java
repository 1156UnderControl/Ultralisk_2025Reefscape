package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Java_Is_UnderControl.LEDs.LedColor;
import frc.Java_Is_UnderControl.Vision.Deprecated.Cameras.LimelightHelpers;
import frc.robot.SuperStructure;
import frc.robot.commands.climber.StowClimber;
import frc.robot.commands.scorer.MoveScorerToDefaultPosition;

public class DefaultPosition extends SequentialCommandGroup {

  public DefaultPosition(SuperStructure superStructure) {
    addCommands(new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceOff("limelight-right")),
        new StowClimber(superStructure),
        new MoveScorerToDefaultPosition(superStructure),
        new InstantCommand(() -> superStructure.led.setRainbow()),
        new InstantCommand(() -> {
          if (superStructure.scorer.hasCoral()) {
            superStructure.led.setSolidColor(LedColor.WHITE);
          }
        }),
        Commands.idle(superStructure));
  }
}
