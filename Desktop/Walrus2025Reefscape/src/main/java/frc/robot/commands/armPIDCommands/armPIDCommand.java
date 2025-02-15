// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armPIDCommands;

import java.security.Timestamp;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class armPIDCommand extends Command {
  ArmSubsystem m_ArmSubsystem;
  Joystick m_TestJoystick;
  private double armEncoderValue;
  private double armEncoderValueTop;
  private double armEncoderValueBottom;
  private double out;
  private double setpoint;
  private double P_Corse;
  private double P_Fine;
  private double D;
  private double engagetime;
  private double timeTop;
  private double timeBottom;
  private boolean timerStart;
  private boolean flagStart;
  private boolean flagEnd;
  
  /** Creates a new armPIDCommand. */

  public armPIDCommand(ArmSubsystem armSubsystem, Joystick joystick) {
    m_ArmSubsystem = armSubsystem;
    addRequirements(m_ArmSubsystem);
    m_TestJoystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setpoint = .185;
    P_Corse = 1.5;
    P_Fine = 3;
    D = 0;
    timerStart = true;
    flagStart = false;
    flagEnd = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  if(flagStart == false && flagEnd == false){
  if (Robot.MasterStagingSensor.get() == true){
    m_ArmSubsystem.m_GrabberMotor.set(.2);
  }
  if (Robot.MasterStagingSensor.get() == false){
    flagStart = true;
  }
  }
  if (flagStart == true && flagEnd == false){
    if (Robot.MasterStagingSensor.get() == false){
      m_ArmSubsystem.m_GrabberMotor.set(.15);
    }
    if (Robot.MasterStagingSensor.get() == true){
      flagEnd = true;
    }
  }
  if (flagStart == true && flagEnd == true){
    if (Robot.MasterStagingSensor.get() == true){
      m_ArmSubsystem.m_GrabberMotor.set(-.1);
    }
    if (Robot.MasterStagingSensor.get() == false){
      m_ArmSubsystem.m_GrabberMotor.set(0);
    }
  }
 // }
     armEncoderValue = Robot.armEncoder.get();
 if( armEncoderValue < .175 || armEncoderValue > .2){
  //System.out.println("Corse Tuning");

    armEncoderValue = Robot.armEncoder.get();
    armEncoderValueTop = Robot.armEncoder.get();
    timeTop = (System.currentTimeMillis()-engagetime)/1000;

    out = ((setpoint - (((armEncoderValue)))) * P_Corse); //(D * ((armEncoderValueBottom)-(armEncoderValueTop))/(timeBottom - timeTop));
    m_ArmSubsystem.m_ArmMotor.set(out);

    armEncoderValueBottom = Robot.armEncoder.get();
 }
 if( armEncoderValue>= .175 && armEncoderValue <= .2){
  //System.out.println("Fine Tuning");
  armEncoderValue = Robot.armEncoder.get();
  armEncoderValueTop = Robot.armEncoder.get();
  timeTop = (System.currentTimeMillis()-engagetime)/1000;

  out = ((setpoint - (((armEncoderValue)))) * P_Fine); //(D * ((armEncoderValueBottom)-(armEncoderValueTop))/(timeBottom - timeTop));
  m_ArmSubsystem.m_ArmMotor.set(out);

 }



    SmartDashboard.putNumber("P output", ((setpoint - ((armEncoderValue)/1000)) * P_Corse));
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
    m_ArmSubsystem.m_GrabberMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
