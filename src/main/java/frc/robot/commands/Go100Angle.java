package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class Go100Angle extends Command{
    ArmSubsystem robot = ArmSubsystem.getInstance();

    public Go100Angle(ArmSubsystem robot) {
      this.robot = robot;
      this.addRequirements(robot);
    }
  
    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
      this.robot.stopMotor();
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}
