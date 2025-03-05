package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.robot.joysticks.ControlBoard;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.IClimber;
import frc.robot.subsystems.intake.IIntake;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.scorer.IScorer;
import frc.robot.subsystems.scorer.ScorerSubsystem;

public class SuperStructure extends SubsystemBase {
  public IScorer scorer;
  public IIntake intake;
  public IClimber climber;

  private ControlBoard controlBoard = ControlBoard.getInstance();

  private PowerDistribution powerDistributionHub;

  private CustomDoubleLogger batteryVoltageLogEntry = new CustomDoubleLogger("/Robot/BatteryVoltage");

  private CustomDoubleLogger totalCurrentDrawLogEntry = new CustomDoubleLogger("/Robot/TotalCurrentDraw");

  public boolean robotIsClimbed = false;

  public SuperStructure() {
    this.scorer = ScorerSubsystem.getInstance();
    this.intake = IntakeSubsystem.getInstance();
    this.climber = ClimberSubsystem.getInstance();
    this.powerDistributionHub = new PowerDistribution();
    this.batteryVoltageLogEntry.append(this.powerDistributionHub.getVoltage());
    this.totalCurrentDrawLogEntry.append(this.powerDistributionHub.getTotalCurrent());
  }

  @Override
  public void periodic() {
    this.scorer.periodic();
    this.intake.periodic();
    this.climber.periodic();
    this.batteryVoltageLogEntry.append(this.powerDistributionHub.getVoltage());
    this.totalCurrentDrawLogEntry.append(this.powerDistributionHub.getTotalCurrent());
  }

  // public boolean scorerHasCoral() {
  // return this.scorer.hasCoral();
  // }

  // public boolean intakeHasCoral() {
  // return this.scorer.hasCoral();
  // }

  public void setCoastToRobot() {
    this.scorer.setCoastScorer();
    this.climber.setCoastClimber();
  }

  public void setBrakeToRobot() {
    this.scorer.setBrakeScorer();
    this.climber.setBrakeClimber();
  }

  public boolean isRobotAbleToScore() {
    // if (this.controlBoard.getDoNotScore()) {
    return false;
    // }
    // return this.scorer.isPivotAtSetPointForAutoScore()
    // && this.scorer.isElevatorAtSetPointForScoring();
    // }
  }
}
