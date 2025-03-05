package frc.robot.commands.teleoperated.teleop_states;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
<<<<<<< HEAD:src/main/java/frc/robot/commands/teleoperated/teleop_states/ClimbPosition.java
import frc.robot.commands.teleoperated.climber.MoveClimberToClimbedPosition;
import frc.robot.commands.teleoperated.climber.MoveClimberToRaisedPosition;
import frc.robot.commands.teleoperated.scorer.MoveScorerToDefaultPosition;
import frc.robot.joysticks.ControlBoard;
import frc.robot.subsystems.swerve.SwerveSubsystem;
=======
import frc.robot.commands.climber.IntakeCageClimber;
import frc.robot.commands.climber.MoveClimberToClimbedPosition;
import frc.robot.commands.climber.MoveClimberToIntakePosition;
import frc.robot.commands.scorer.MoveScorerToDefaultPositionWithCoral;
import frc.robot.joysticks.OperatorController;
>>>>>>> main:src/main/java/frc/robot/commands/states/ClimbPosition.java

public class ClimbPosition extends SequentialCommandGroup {
  OperatorController operatorKeyboard = OperatorController.getInstance();

  public ClimbPosition(SuperStructure superStructure) {
    addCommands(new MoveScorerToDefaultPositionWithCoral(superStructure),
        new MoveClimberToIntakePosition(superStructure),
        new IntakeCageClimber(superStructure),
        Commands.waitUntil(operatorKeyboard.climb()),
        new MoveClimberToClimbedPosition(superStructure),
        new InstantCommand(() -> superStructure.robotIsClimbed = true),
        Commands.idle(superStructure));
  }
}
