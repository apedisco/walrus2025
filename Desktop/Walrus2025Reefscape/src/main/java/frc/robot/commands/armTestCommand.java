// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class armTestCommand extends Command {
  ArmSubsystem m_ArmSubsystem;
  Joystick m_ElevatorJoystick;
  Joystick m_TestJoystick;
  /** Creates a new armTestCommand. */
  public armTestCommand(ArmSubsystem armSubsystem, Joystick elevatorJoystick, Joystick testJoystick) {
    m_ArmSubsystem = armSubsystem;
    addRequirements(m_ArmSubsystem);
    m_ElevatorJoystick = elevatorJoystick;
    m_TestJoystick = testJoystick;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ArmSubsystem.m_GrabberMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ArmSubsystem.m_GrabberMotor.set((((m_ElevatorJoystick.getRawAxis(3) + 1) / 2)*.5));
    //m_ArmSubsystem.m_ArmMotor.set((((m_ElevatorJoystick.getRawAxis(3) + 1) / 2)*.5));
    // if(Robot.MasterStagingSensor.get() == true){
    //   m_ArmSubsystem.m_GrabberMotor.set(.3);
    // }
    // if(Robot.MasterStagingSensor.get() == false){
    // m_ArmSubsystem.m_GrabberMotor.set(0);
    // }
    // SmartDashboard.putBoolean("Master Staging Sensor", Robot.MasterStagingSensor.get());
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ArmSubsystem.m_ArmMotor.set(0);
    m_ArmSubsystem.m_GrabberMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
