// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomPose3dLogger;
import frc.Java_Is_UnderControl.Util.AllianceFlipUtil;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.ReefHeight;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  CustomPose3dLogger logPoses = new CustomPose3dLogger("pose reef");

  CustomPose3dLogger logPosesred = new CustomPose3dLogger("pose reef red");

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    PathfindingCommand.warmupCommand().schedule();
  }

  @Override
  public void robotPeriodic() {
    Pose3d pose = AllianceFlipUtil.apply(Reef.branchPositions.get(0).get(ReefHeight.L2));
    logPosesred.appendRadians(pose.transformBy(pose.minus(new Pose3d(1, 1, 0, new Rotation3d()))));
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
