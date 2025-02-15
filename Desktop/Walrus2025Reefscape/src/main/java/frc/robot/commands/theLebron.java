// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class theLebron extends Command {
  ArmSubsystem m_ArmSubsystem;
  private double engagetime;
  private double setpoint;
  private double armEncoderValue;
  private double armEncoderValueTop;
  private double timeTop;
  private double out;
  private double armEncoderValueBottom;
  private double timeBottom;
  private double P;
  private boolean armStop;

  /** Creates a new theLebron. */
  public theLebron(ArmSubsystem armSubsystem) {
    m_ArmSubsystem = armSubsystem;
    addRequirements(m_ArmSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armStop = true;
    engagetime = System.currentTimeMillis();
    setpoint = .22;
    //P = 2.5;// .5
    System.out.println("Lebron started");
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((System.currentTimeMillis() - engagetime) < 160 && armStop){

    armEncoderValue = Robot.armEncoder.get();
    armEncoderValueTop = Robot.armEncoder.get();
    timeTop = (System.currentTimeMillis()-engagetime)/1000;

    out = ((setpoint - (((armEncoderValue)))) * P); //(D * ((armEncoderValueBottom)-(armEncoderValueTop))/(timeBottom - timeTop));
    m_ArmSubsystem.m_ArmMotor.set(-1);

    armEncoderValueBottom = Robot.armEncoder.get();
    timeBottom = (System.currentTimeMillis()-engagetime)/1000;

    System.out.println(out);

    }
    if((System.currentTimeMillis() - engagetime) > 160){
      m_ArmSubsystem.m_GrabberMotor.set(-.95);
      m_ArmSubsystem.m_ArmMotor.set(0);
      armStop = false;
    }
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
