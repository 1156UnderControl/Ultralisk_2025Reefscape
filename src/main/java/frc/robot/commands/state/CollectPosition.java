package frc.robot.commands.state;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class CollectPosition extends SequentialCommandGroup {

  public CollectPosition(SuperStructure superStructure, SwerveSubsystem swerve) {
    addCommands();
  }
}
