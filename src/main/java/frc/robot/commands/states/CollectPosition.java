package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Java_Is_UnderControl.LEDs.LedColor;
import frc.robot.SuperStructure;
import frc.robot.commands.scorer.MoveScorerToCollectPosition;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class CollectPosition extends SequentialCommandGroup {

  public CollectPosition(SuperStructure superStructure, SwerveSubsystem swerve) {
    addCommands(new InstantCommand(() -> superStructure.led.setSolidColor(LedColor.YELLOW)),
        new MoveScorerToCollectPosition(superStructure),
        new InstantCommand(() -> superStructure.led.setBlink(LedColor.GREEN, 3, LedColor.WHITE)),
        Commands.waitSeconds(0.6));
  }
}
