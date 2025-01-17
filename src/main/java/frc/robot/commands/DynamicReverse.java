package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class DynamicReverse extends Command {
    private ArmSubsystem robot;
  
    public DynamicReverse(ArmSubsystem robot) {
      this.robot = robot;
      this.addRequirements(robot);
    }
  
    @Override
    public void execute() {
      this.robot.sysIdQuasistaticForward();
    }
  
    @Override
    public boolean isFinished() {
      return this.robot.isAtSetPoint();
    }
}