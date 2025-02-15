// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.L_ScoreCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L4ScoreCommand extends Command {
  ArmSubsystem m_ArmSubsystem;
  ElevatorSubsystem m_ElevatorSubsystem;
private double armEncoderValue;
private double out;
private double armSetPoint;
private double elevatorSetPoint;
private double engagetime;
private double elevatorEncoderValue;
private double elevatorEncoderValueCheck;
private boolean dropcheck;
private double armP;
private boolean armOn;
private double elevatorPAccelerate;
private double elevatorPConstant;
private double elevatorPPrecise;
  /** Creates a new L3ScoreCommand. */
  public L4ScoreCommand(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem) {
    m_ArmSubsystem = armSubsystem;
    addRequirements(m_ArmSubsystem);
    m_ElevatorSubsystem = elevatorSubsystem;
    addRequirements(m_ElevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSetPoint = .3;
    elevatorSetPoint = -2;
    armP = .9;
    elevatorPAccelerate = .5;
    elevatorPConstant = .1;
    elevatorPPrecise = .2;
    dropcheck = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Timer Start", dropcheck);
    armEncoderValue = Robot.armEncoder.get();

 if (armEncoderValue < .30 || armEncoderValue > .36 && dropcheck == true){
   System.out.println("armPID Running");
   armEncoderValue = Robot.armEncoder.get();

   out = ((armSetPoint - (((armEncoderValue)))) * armP); //(D * ((armEncoderValueBottom)-(armEncoderValueTop))/(timeBottom - timeTop));
   m_ArmSubsystem.m_ArmMotor.set(out);
 }

 if (armEncoderValue > .26 && armEncoderValue < .3 && dropcheck == true){

   armEncoderValue = Robot.armEncoder.get();

   out = ((armSetPoint - (((armEncoderValue)))) * armP); //(D * ((armEncoderValueBottom)-(armEncoderValueTop))/(timeBottom - timeTop));
   m_ArmSubsystem.m_ArmMotor.set(out);

   elevatorEncoderValue = Robot.elevatorEncoder.get();
   System.out.println("Business as usual");

   if(elevatorEncoderValue <= .5){
   out = (0) + ((((elevatorEncoderValue)/1000)+elevatorSetPoint)* elevatorPAccelerate); // ((0.000001 * (((-elevatorEncoderValue)/1000) - ((-elevatorEncoderValueCheck)/1000))) / ((System.currentTimeMillis()-engagetime)/1000));
   m_ElevatorSubsystem.m_ElevatorMotor1.set(-out);
   m_ElevatorSubsystem.m_ElevatorMotor2.set(-out);
   System.out.println("Accelerating");
   }
   if(elevatorEncoderValue > .5 && elevatorEncoderValue < 1.75){
     out = (0) + ((((elevatorEncoderValue)/1000) + elevatorSetPoint) * elevatorPConstant); // ((0.000001 * (((-elevatorEncoderValue)/1000) - ((-elevatorEncoderValueCheck)/1000))) / ((System.currentTimeMillis()-engagetime)/1000));
   m_ElevatorSubsystem.m_ElevatorMotor1.set(-out);
   m_ElevatorSubsystem.m_ElevatorMotor2.set(-out);
    System.out.println("Constant");
   }
   if(elevatorEncoderValue >= 1.75){
    out = (0) + ((((elevatorEncoderValue)/1000) + elevatorSetPoint) * elevatorPPrecise); // ((0.000001 * (((-elevatorEncoderValue)/1000) - ((-elevatorEncoderValueCheck)/1000))) / ((System.currentTimeMillis()-engagetime)/1000));
   m_ElevatorSubsystem.m_ElevatorMotor1.set(-out);
   m_ElevatorSubsystem.m_ElevatorMotor2.set(-out);
   System.out.println("Precise");
   }
   elevatorEncoderValueCheck = Robot.elevatorEncoder.get();
   System.out.println((elevatorEncoderValueCheck/1000));
   if(-(elevatorEncoderValueCheck/1000) < -2.9 & dropcheck == true){
     engagetime = System.currentTimeMillis();
     dropcheck = false;
     System.out.println("Timer Start");
   }
 }
 if(dropcheck == false && System.currentTimeMillis() - engagetime <300){
   m_ArmSubsystem.m_GrabberMotor.set(.4); 
   m_ArmSubsystem.m_ArmMotor.set(0);
   System.out.println(System.currentTimeMillis() - engagetime);
 }
 if(armOn == false && dropcheck == false && System.currentTimeMillis() - engagetime >=300){
   m_ArmSubsystem.m_GrabberMotor.set(0); //&& System.currentTimeMillis() < 600
   m_ElevatorSubsystem.m_ElevatorMotor1.set(0);
   m_ElevatorSubsystem.m_ElevatorMotor2.set(0);
   elevatorEncoderValue = Robot.elevatorEncoder.get();
 }
 if (dropcheck == false && (elevatorEncoderValue/1000) > -.3){
   System.out.println("armPID Running");
   armEncoderValue = Robot.armEncoder.get();

   out = ((.22 - (((armEncoderValue)))) * armP); //(D * ((armEncoderValueBottom)-(armEncoderValueTop))/(timeBottom - timeTop));
   m_ArmSubsystem.m_ArmMotor.set(out);

   armOn = false;
 }
   SmartDashboard.putNumber("P output", ((armSetPoint - ((armEncoderValue)/1000)) * armP));
   SmartDashboard.putNumber("Arm Encoder", armEncoderValue);
   SmartDashboard.putNumber("Power out", out);
   SmartDashboard.putNumber("Timer", ((System.currentTimeMillis()-engagetime)/1000));
   SmartDashboard.putNumber("delta Setpoint", (armSetPoint - (((armEncoderValue)))));
   SmartDashboard.putNumber("Elevator Encoder Value", (-elevatorEncoderValue)/1000);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
