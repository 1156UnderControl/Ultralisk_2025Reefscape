package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.LEDs.ILed;
import frc.Java_Is_UnderControl.LEDs.LedSubsystem;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomStringLogger;
import frc.robot.joysticks.ControlBoard;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.IClimber;
import frc.robot.subsystems.scorer.IScorer;
import frc.robot.subsystems.scorer.ScorerSubsystem;

public class SuperStructure extends SubsystemBase {
  public IScorer scorer;
  public IClimber climber;
  public ILed led;

  private ControlBoard controlBoard = ControlBoard.getInstance();

  private PowerDistribution powerDistributionHub;

  private CustomStringLogger autoStateLogger = new CustomStringLogger("/Robot/AutoState");

  private CustomDoubleLogger batteryVoltageLogEntry = new CustomDoubleLogger("/Robot/BatteryVoltage");

  private CustomDoubleLogger totalCurrentDrawLogEntry = new CustomDoubleLogger("/Robot/TotalCurrentDraw");

  public boolean robotIsClimbed = false;

  public String auto_State = "NONE";

  public SuperStructure() {
    this.scorer = ScorerSubsystem.getInstance();
    this.climber = ClimberSubsystem.getInstance();
    this.led = LedSubsystem.getInstance();
    this.powerDistributionHub = new PowerDistribution();
    this.batteryVoltageLogEntry.append(this.powerDistributionHub.getVoltage());
    this.totalCurrentDrawLogEntry.append(this.powerDistributionHub.getTotalCurrent());
  }

  @Override
  public void periodic() {
    this.scorer.periodic();
    this.climber.periodic();
    this.autoStateLogger.append(this.auto_State);
    this.batteryVoltageLogEntry.append(this.powerDistributionHub.getVoltage());
    this.totalCurrentDrawLogEntry.append(this.powerDistributionHub.getTotalCurrent());
  }

  public boolean scorerHasCoral() {
    return this.scorer.hasCoral();
  }

  public void setCoastToRobot() {
    this.scorer.setCoastScorer();
    this.climber.setCoastClimber();
  }

  public void setBrakeToRobot() {
    this.scorer.setBrakeScorer();
    this.climber.setBrakeClimber();
  }

}
