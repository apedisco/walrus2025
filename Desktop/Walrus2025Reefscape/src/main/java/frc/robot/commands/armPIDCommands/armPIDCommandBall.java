// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armPIDCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class armPIDCommandBall extends Command {
  ArmSubsystem m_ArmSubsystem;
  private double armEncoderValue;
  private double armEncoderValueTop;
  private double armEncoderValueBottom;
  private double out;
  private double setpoint;
  private double P;
  private double D;
  private double engagetime;
  private double timeTop;
  private double timeBottom;
  
  /** Creates a new armPIDCommand. */
  public armPIDCommandBall(ArmSubsystem armSubsystem) {
    m_ArmSubsystem = armSubsystem;
    addRequirements(m_ArmSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    engagetime = System.currentTimeMillis();
    setpoint = .58;
    P = 1.5;
    D = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("armPID Running");
    armEncoderValue = Robot.armEncoder.get();
    armEncoderValueTop = Robot.armEncoder.get();
    timeTop = (System.currentTimeMillis()-engagetime)/1000;

    out = ((setpoint - (((armEncoderValue)))) * P); //(D * ((armEncoderValueBottom)-(armEncoderValueTop))/(timeBottom - timeTop));
    m_ArmSubsystem.m_ArmMotor.set(out);

    armEncoderValueBottom = Robot.armEncoder.get();
    timeBottom = (System.currentTimeMillis()-engagetime)/1000;

    SmartDashboard.putNumber("P output", ((setpoint - ((armEncoderValue)/1000)) * P));
    SmartDashboard.putNumber("D output",(D * ((armEncoderValueBottom)-(armEncoderValueTop))/(timeBottom - timeTop))) ;
    SmartDashboard.putNumber("Arm Encoder", armEncoderValue);
    SmartDashboard.putNumber("Power out", out);
    SmartDashboard.putNumber("Timer", ((System.currentTimeMillis()-engagetime)/1000));
    SmartDashboard.putNumber("delta Setpoint", (setpoint - (((armEncoderValue)))));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ArmSubsystem.m_ArmMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
