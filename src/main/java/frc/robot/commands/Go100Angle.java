package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.test_subsystems.ArmSubsystem;

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
      System.out.println("TO RODANDO");
      this.robot.go100Degrees();
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}
