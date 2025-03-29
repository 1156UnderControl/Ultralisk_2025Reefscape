package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Java_Is_UnderControl.LEDs.LedColor;
import frc.robot.SuperStructure;
import frc.robot.commands.scorer.MoveScorerToRemovePosition;
import frc.robot.joysticks.ControlBoard;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RemoveAlgaePosition extends SequentialCommandGroup {
  ControlBoard controlBoard = ControlBoard.getInstance();

  public RemoveAlgaePosition(SuperStructure superStructure, SwerveSubsystem swerve) {
    addCommands(new MoveScorerToRemovePosition(superStructure));
    Commands.runOnce(() -> superStructure.led.setSolidColor(LedColor.RED));
  }
}
