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
public class L3ScoreCommand extends Command {
    ArmSubsystem m_ArmSubsystem;
    ElevatorSubsystem m_ElevatorSubsystem;
  private double armEncoderValue;
  private double out;
  private double setPointScoring;
  private double setPointIntaking;
  // private double setPointTransporting; not used for L3, leave for now
  private double engagetime;
  private double elevatorEncoderValue;
  private double elevatorEncoderValueCheck;
  private boolean dropCheck;
  private boolean timerStart;
  private double armP;
  private boolean armOn;
  private double elevatorPAccelerate;
  private double elevatorPConstant;
  //private double elevatorPPrecise;

  /** Creates a new L3ScoreCommand. */
  public L3ScoreCommand(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem) {
    m_ArmSubsystem = armSubsystem;
    addRequirements(m_ArmSubsystem);
    m_ElevatorSubsystem = elevatorSubsystem;
    addRequirements(m_ElevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
// Arm Constants
    setPointIntaking = .185;
    setPointScoring = .3;
    //setPointTransporting = .25; not used for L3, leave for now
    armP = .9;

// Elevator Constants
    elevatorPAccelerate = .1;
    elevatorPConstant = .2;
    //elevatorPPrecise = .1;

// Boolean Flags
    dropCheck = true;
    timerStart = true;
  }

// Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  SmartDashboard.putBoolean("Timer Start", dropCheck);
  armEncoderValue = Robot.armEncoder.get();
  
// Looks to See if the Arm is outside of the acceptable range and if it is the arm PID is set to setPointScoring
    if (armEncoderValue < .26 || armEncoderValue > .3 && dropCheck == true){
    //System.out.println("Arm PID setPointScoring");

      armEncoderValue = Robot.armEncoder.get();
      out = ((setPointScoring - (((armEncoderValue)))) * armP);
      m_ArmSubsystem.m_ArmMotor.set(out);
    }

// If the Arm is Within the Acceptable Range then run the elevator
    if (armEncoderValue > .26 && armEncoderValue < .3 && dropCheck == true){
    System.out.println(timerStart);
    //System.out.println("Arm PID setPointScoring");
      armEncoderValue = Robot.armEncoder.get();
  // Arm PID output
     out = ((setPointScoring - (((armEncoderValue)))) * armP);
     m_ArmSubsystem.m_ArmMotor.set(out);
  
  
      elevatorEncoderValue = Robot.elevatorEncoder.get();
       //System.out.println("Arm PID setPointScoring && Elevator Running");

  // If the Elevator is under <= one rotation accelerate the motors 
      if(elevatorEncoderValue <= 1){
      out = (0) + (((((elevatorEncoderValue)/1000+-3.2)))* elevatorPAccelerate); // Leave for Legacy purposes ((0.000001 * (((-elevatorEncoderValue)/1000) - ((-elevatorEncoderValueCheck)/1000))) / ((System.currentTimeMillis()-engagetime)/1000));
      m_ElevatorSubsystem.m_ElevatorMotor1.set(-out);
      m_ElevatorSubsystem.m_ElevatorMotor2.set(-out);
       //System.out.println("Arm PID setPointScoring && Elevator PAccelerating");
     }
      if(elevatorEncoderValue > 1){
        out = (0) + (((((elevatorEncoderValue)/1000+-3.2)))* elevatorPConstant);
      m_ElevatorSubsystem.m_ElevatorMotor1.set(-out);
      m_ElevatorSubsystem.m_ElevatorMotor2.set(-out);
       //System.out.println("Arm Pid setPointScoring && Elevator PConstant");
      }
     elevatorEncoderValueCheck = Robot.elevatorEncoder.get();
      System.out.println((elevatorEncoderValueCheck/1000));
      if((-elevatorEncoderValueCheck/1000) < -2.6 && timerStart == true){
       engagetime = System.currentTimeMillis();
       timerStart = false;
     System.out.println("Timer Start");
     }
    }

    if(timerStart == false && System.currentTimeMillis() - engagetime <300){
      m_ArmSubsystem.m_GrabberMotor.set(.4); 
      m_ArmSubsystem.m_ArmMotor.set(0);
      System.out.println(System.currentTimeMillis() - engagetime);
    }
    if(armOn == false && timerStart == false && System.currentTimeMillis() - engagetime >=300){
      dropCheck = true;
      m_ArmSubsystem.m_GrabberMotor.set(0); //&& System.currentTimeMillis() < 600
      m_ElevatorSubsystem.m_ElevatorMotor1.set(0);
      m_ElevatorSubsystem.m_ElevatorMotor2.set(0);
      elevatorEncoderValue = Robot.elevatorEncoder.get();
    }
    if (dropCheck == false && (elevatorEncoderValue/1000) > -.3){
     System.out.println("armPID Running");
     armEncoderValue = Robot.armEncoder.get();

      out = ((setPointIntaking - (((armEncoderValue)))) * armP); //(D * ((armEncoderValueBottom)-(armEncoderValueTop))/(timeBottom - timeTop));
      m_ArmSubsystem.m_ArmMotor.set(out);

      armOn = false;
    }
      SmartDashboard.putNumber("P output", ((setPointScoring - ((armEncoderValue)/1000)) * armP));
      SmartDashboard.putNumber("Arm Encoder", armEncoderValue);
      SmartDashboard.putNumber("Power out", out);
      SmartDashboard.putNumber("Timer", ((System.currentTimeMillis()-engagetime)/1000));
      SmartDashboard.putNumber("delta Setpoint", (setPointScoring - (((armEncoderValue)))));
      SmartDashboard.putNumber("Elevator Encoder Value", (-elevatorEncoderValue)/1000);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ElevatorSubsystem.elevatorMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
