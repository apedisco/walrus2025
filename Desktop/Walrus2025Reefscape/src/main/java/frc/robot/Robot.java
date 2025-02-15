// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Array;

import com.revrobotics.jni.DistanceSensorJNIWrapper;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// import com.revrobotics.Rev2mDistanceSensor;
// import com.revrobotics.Rev2mDistanceSensor.Port;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  

  public DoubleLogEntry motor1Position;

  private final RobotContainer m_robotContainer;
  ArmSubsystem m_ArmSubsystem;
  ElevatorSubsystem m_ElevatorSubsystem;

  public double l_ATag;
  public double r_ATag;
  public double r_limeLightMountingDegrees = 0;
  public double r_limeLightHeightOffset = 9.8125;
  public double goalHeightInches = 12.125;
  public double r_angleToGoalDegrees;
  public double r_angleToGoalRadians;
  public double r_distanceFromLimelightToGoalInches;

  public static DigitalInput input0 = new DigitalInput(0);
  public static DigitalInput input1 = new DigitalInput(1);
  public static DigitalInput input2 = new DigitalInput(2);
  public static DigitalInput input3 = new DigitalInput(3);
  public static DigitalInput MasterStagingSensor = new DigitalInput(4);
  public static DigitalInput input5 = new DigitalInput(5);
  public static DigitalInput input6 = new DigitalInput(6);
  public static DigitalInput input7 = new DigitalInput(7);
  public static DigitalInput input8 = new DigitalInput(8);
  public static DigitalInput input9 = new DigitalInput(9);
 
  public static Encoder elevatorEncoder = new Encoder(input0, input1);
  public static DutyCycleEncoder armEncoder = new DutyCycleEncoder(input5);
  // public static Rev2m
  //public static Encoder armEncoder = new Encoder(input2, input3);

  // private Rev2mDistanceSensor distOnboard; 
  // private Rev2mDistanceSensor distMXP;

  public Robot() {
    m_robotContainer = new RobotContainer();
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLog log = DataLogManager.getLog();

    motor1Position = new DoubleLogEntry(log, "Motor 1 Encoder position");
  }

  @Override
  public void robotPeriodic() {
  SmartDashboard.putNumber("Elevator Encoder Value", elevatorEncoder.get());
  SmartDashboard.putNumber("Arm Encoder Value", armEncoder.get());
    CommandScheduler.getInstance().run(); 

    //System.out.println(input8.get());
 
  // Networktable and variables for the right Limelight
  NetworkTable r_table = NetworkTableInstance.getDefault().getTable("limelight-right"); 

  NetworkTableEntry r_cam3d = r_table.getEntry("camerapose_targetspace");
  NetworkTableEntry r_tx = r_table.getEntry("tx");
  NetworkTableEntry r_ty = r_table.getEntry("ty");
  NetworkTableEntry r_ta = r_table.getEntry("ta");
  NetworkTableEntry r_tid = r_table.getEntry("tid");
  NetworkTableEntry r_tv = r_table.getEntry("tv");
  //NetworkTableEntry r_ry = r_table.getEntry("ry");
  //NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDoubleArray(new double[6]);
  //read values periodically
  double r_sx = r_tx.getDouble(0.0);
  double r_sy = r_ty.getDouble(0.0);
  double r_area = r_ta.getDouble(0.0);
  double r_aid = r_tid.getDouble(0);
  double[] r_rotation = r_cam3d.getDoubleArray(new double[6]);
  r_ATag = r_tv.getDouble(0);
  SmartDashboard.putNumber("r_X-Offset", r_sx);
  SmartDashboard.putNumber("ATag Detector", r_ATag);
  SmartDashboard.putNumber("Limelight", r_sx);
  SmartDashboard.putNumber("ATag Area", r_area);
  SmartDashboard.putNumber("rotation", r_rotation[4]);

  r_angleToGoalDegrees = r_limeLightMountingDegrees + r_sy;
  r_angleToGoalRadians = r_angleToGoalDegrees * (3.14159 / 180);

  r_distanceFromLimelightToGoalInches = (goalHeightInches - r_limeLightHeightOffset) / Math.tan(r_angleToGoalRadians);
  SmartDashboard.putNumber("Distance from Limelight-right to ATag", r_distanceFromLimelightToGoalInches);

  // Networktable and variables for the left Limelight
  NetworkTable l_table = NetworkTableInstance.getDefault().getTable("limelight-left");
  
  NetworkTableEntry l_cam3d = l_table.getEntry("camerapose_targetspace");
  NetworkTableEntry l_tx = l_table.getEntry("tx");
  NetworkTableEntry l_ty = l_table.getEntry("ty");
  NetworkTableEntry l_ta = l_table.getEntry("ta");
  NetworkTableEntry l_tid = l_table.getEntry("tid");
  NetworkTableEntry l_tv = l_table.getEntry("tv");
  //NetworkTableEntry r_ry = l_table.getEntry("ry");
  //NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDoubleArray(new double[6]);
  //read values periodically
  double l_sx = l_tx.getDouble(0.0);
  double l_sy = l_ty.getDouble(0.0);
  double l_area = l_ta.getDouble(0.0);
  double l_aid = l_tid.getDouble(0);
  double[] rotation = l_cam3d.getDoubleArray(new double[6]);
  l_ATag = r_tv.getDouble(0);
  SmartDashboard.putNumber("ATag Detector", l_ATag);
  SmartDashboard.putNumber("Limelight", l_sx);
  SmartDashboard.putNumber("ATag Area", l_area);
  SmartDashboard.putNumber("rotation", rotation[4]);
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

    //  if(distOnboard.isRangeValid()) {
    //   SmartDashboard.putNumber("Range Onboard", distOnboard.getRange());
    //   SmartDashboard.putNumber("Timestamp Onboard", distOnboard.getTimestamp());
    // }

    // if(distMXP.isRangeValid()) {
    //   SmartDashboard.putNumber("Range MXP", distMXP.getRange());
    //   SmartDashboard.putNumber("Timestamp MXP", distMXP.getTimestamp());
    // }


  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
