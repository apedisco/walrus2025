// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.L_ScoreCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L2ScoreCommand extends Command {
  ArmSubsystem m_ArmSubsystem;
  private double armEncoderValue;
  private double out;
  private double setPointScoring;
  private double setPointIntaking;
  private double P;
  private double engagetime;
  private boolean armStop;
  private boolean stopCheck;

  /** Creates a new L2ScoreCommand. */
  public L2ScoreCommand(ArmSubsystem armSubsystem) {
    m_ArmSubsystem = armSubsystem;
    addRequirements(m_ArmSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    engagetime = System.currentTimeMillis();
    setPointScoring = .27; // The point that the arm curls to when scoring
    setPointIntaking = .19; // The point that the arm curls to when intaking
    P = 1.75;
    stopCheck = true; // used to start the timer
    armStop = false; // used to shut off the setPointScoring armPID 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  armEncoderValue = Robot.armEncoder.get(); // this should always be running no matter what

// Tells the arm to move to the scoring position
  if (armStop == false){
    System.out.println("Arm PID setPointScoring");
    armEncoderValue = Robot.armEncoder.get();
    out = ((setPointScoring - (((armEncoderValue)))) * P); 
    m_ArmSubsystem.m_ArmMotor.set(out);
  }

// Looks for if the arm is close to the scoring position and when true starts the timer
  if(armEncoderValue > .25 && armEncoderValue < .3 && stopCheck){ //correct position, 1st instance
    System.out.println("Start the Timer");
    engagetime = System.currentTimeMillis();
    stopCheck = false;
  }

// Once it is in the right postion the shoot motors turn on and shoot the coral out as long as the time is <= .3 seconds
  if(armEncoderValue > .25 && armEncoderValue < .3 && (System.currentTimeMillis() - engagetime <= 300 )){ //correct position, shoot
    m_ArmSubsystem.m_GrabberMotor.set(.3);
    System.out.println("Shooting");
  }

// Once the time is > .3 seconds the arm scoring PID shuts off and the arm Intaking PID turns on
  if(System.currentTimeMillis() - engagetime > 300 && stopCheck == false){
    m_ArmSubsystem.m_GrabberMotor.set(0);
    armStop = true;
    System.out.println("Arm PID setPositionIntaking");
    armEncoderValue = Robot.armEncoder.get();
    
    out = ((setPointIntaking - (((armEncoderValue)))) * P); 
    m_ArmSubsystem.m_ArmMotor.set(out);
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ArmSubsystem.m_GrabberMotor.set(0);
    m_ArmSubsystem.m_ArmMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
