// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.controls.SolidColor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private StructLogEntry<Pose2d> poseLog;
  private StructLogEntry<Pose3d> ComponentPoseZero;
  private StructLogEntry<Pose3d> ComponentPoseFinal;


  public Robot() {
    m_robotContainer = new RobotContainer();
  }
  @Override
  public void robotInit() {
    try{
     
      m_robotContainer.music.schedule();
      } catch (Exception e){
          System.out.println("Failed to play music" + e.getMessage());
          e.printStackTrace();
      }

      DataLogManager.start("/media/sda1");
      DataLog log = DataLogManager.getLog();
      poseLog = StructLogEntry.create(log, "/drivetrain/pose", Pose2d.struct);
      ComponentPoseZero = StructLogEntry.create(log, "/drivetrain/ComponentPoseZero", Pose3d.struct);
    ComponentPoseFinal = StructLogEntry.create(log, "/drivetrain/ComponentPoseFinal", Pose3d.struct);
      DriverStation.startDataLog(DataLogManager.getLog());
   
  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
   
  }

  @Override
  public void teleopPeriodic() {
    Pose2d robotPose = m_robotContainer.drivetrain.getState().Pose;
    poseLog.append(robotPose);
    ComponentPoseZero.append(new Pose3d());
    ComponentPoseFinal.append(new Pose3d(0, 0.0, 0.82, new Rotation3d(0.0, Math.sin(Timer.getTimestamp()) - 1.0, 0.0)));
  }

  @Override
  public void teleopExit() {
    DataLogManager.stop();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    
    
  }
}
