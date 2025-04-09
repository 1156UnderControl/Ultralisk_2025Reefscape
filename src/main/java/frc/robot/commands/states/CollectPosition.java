package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Java_Is_UnderControl.LEDs.LedColor;
import frc.robot.SuperStructure;
import frc.robot.commands.scorer.MoveScorerToCollectPosition;
import frc.robot.subsystems.leds.LedSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class CollectPosition extends SequentialCommandGroup {

  public CollectPosition(SuperStructure superStructure, SwerveSubsystem swerve) {
    addCommands(
        new MoveScorerToCollectPosition(superStructure),
        new RunCommand(() -> {
          if (superStructure.scorer.isAtCollectCoralPosition()) {
            superStructure.led.setBlink(LedColor.YELLOW);
          }
        }, LedSubsystem.getInstance()).withTimeout(0.6));
  }
}
