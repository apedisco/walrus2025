// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  
  public TalonFX m_ElevatorMotor1 = new TalonFX(11);
  public TalonFX m_ElevatorMotor2 = new TalonFX(12);
  TalonFXConfiguration toConfigure = new TalonFXConfiguration();
  public int MasterID = 9;
  public boolean OpposeMasterDirection = false;
  public Follower motor2 = new Follower(MasterID, OpposeMasterDirection);
  public double ElevatorMotor1Position;
  public double ElevatorMotor2Position;
  //public StatusCode setControl(Follower request);
  
  
  

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    // m_ElevatorMotor1.setInverted(true);
    // m_ElevatorMotor2.setInverted(true);
    m_ElevatorMotor1.setPosition(0);
    m_ElevatorMotor2.setPosition(0);
    

  

  toConfigure.Slot0.kS = 0.00; // Add 0.1 V output to overcome static friction
  toConfigure.Slot0.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
  toConfigure.Slot0.kP = .11; // An error of 1 rps results in 0.11 V output
  toConfigure.Slot0.kI = 0; // no output for integrated error
  toConfigure.Slot0.kD = 0.1; // no output for error derivative

  // m_ElevatorMotor1.getConfigurator().apply(toConfigure);

  ElevatorMotor1Position = m_ElevatorMotor1.getRotorPosition().getValueAsDouble();
  ElevatorMotor2Position = m_ElevatorMotor2.getRotorPosition().getValueAsDouble();
  SmartDashboard.putNumber("Elevator Motor 1 Position", ElevatorMotor1Position);
  SmartDashboard.putNumber("Elevator Motor 2 Position", ElevatorMotor2Position);
  }
  public void elevatorVoltageOut(){
    m_ElevatorMotor1.getMotorVoltage().getValueAsDouble();
  }
  public void elevatorMotors(double power){
    m_ElevatorMotor1.set(power);
    m_ElevatorMotor2.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
