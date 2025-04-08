package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Java_Is_UnderControl.LEDs.LedColor;
import frc.robot.SuperStructure;
import frc.robot.commands.scorer.MoveScorerToCollectPosition;
import frc.robot.commands.swerve.SwerveAngleWithCoralStation;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class CollectPosition extends SequentialCommandGroup {

  public CollectPosition(SuperStructure superStructure, SwerveSubsystem swerve) {
    new InstantCommand(() -> superStructure.led.setSolidColor(LedColor.YELLOW));
    addCommands(Commands.race(new MoveScorerToCollectPosition(superStructure), new SwerveAngleWithCoralStation(swerve)),
        new InstantCommand(() -> superStructure.led.setSolidColor(LedColor.PURPLE)),
        Commands.waitSeconds(0.6));
  }
}
