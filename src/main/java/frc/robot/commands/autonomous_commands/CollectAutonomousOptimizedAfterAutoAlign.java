package frc.robot.commands.autonomous_commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructure;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class CollectAutonomousOptimizedAfterAutoAlign extends Command {
  SuperStructure superStructure;
  Timer collectTimer;
  SwerveSubsystem swerve;
  boolean timerFinished = false;

  public CollectAutonomousOptimizedAfterAutoAlign(SuperStructure superStructure, SwerveSubsystem swerve) {
    this.swerve = swerve;
    this.superStructure = superStructure;
    addRequirements(this.superStructure);
    collectTimer = new Timer();
  }

  @Override
  public void initialize() {
    this.superStructure.auto_State = "COLLECTING";
  }

  @Override
  public void execute() {
    if (!this.swerve.isAtTargetPositionWithoutHeading()) {
      superStructure.scorer.intakeFromHP();
    } else {
      collectTimer.start();
      if (collectTimer.get() < 2) {
        superStructure.scorer.intakeFromHP();
      } else {
        if (superStructure.scorerHasCoral()) {
          superStructure.scorer.setTimerAfterAutoAlignFinished(false);
        } else {
          this.superStructure.scorer.stopEndEffector();
          superStructure.scorer.setTimerAfterAutoAlignFinished(true);
        }
      }
    }
    superStructure.scorer.intakeFromHP();
  }

  @Override
  public boolean isFinished() {
    return superStructure.scorer.hasCoral() || superStructure.scorer.isTimerAfterAutoAlignFinished();
  }

  @Override
  public void end(boolean interrupted) {
    collectTimer.stop();
    collectTimer.reset();
    System.out.println("Terminei After alignnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn");
  }
}
