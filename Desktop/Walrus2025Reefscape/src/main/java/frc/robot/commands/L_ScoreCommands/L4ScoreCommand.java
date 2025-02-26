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
  private double elevatorOut;
  private double armOut;
  private double armSetPointTraveling;
  private double armSetPointIntaking;
  private double armSetPointScoring;
  private double elevatorSetPointScoring;
  // private double setPointTransporting; not used for L3, leave for now
  private double pullbackTime;
  private double engagetime;
  private double elevatorEncoderValue;
  private double elevatorEncoderValueCheck;
  private boolean dropCheck;
  private boolean timerStart;
  private boolean stage1;
  private boolean stage2;
  private boolean pullback;
  private boolean pullbackTimerStart;
  private double armP;
  private boolean armOn;
  private double elevatorPAccelerate;
  private double elevatorPConstant;
  private boolean check1;
  private boolean check2;
  private boolean check3;
  private boolean check4;
  private boolean check5;
  private boolean check6;
  private boolean check7;
  //private double elevatorPPrecise;

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
    check1 = false;
    check2 = false;
    check3 = false;
    check4 = false;
    check5 = false;
    check6 = false;
    check7 = false;

// Arm Constants
    armSetPointIntaking = .185;
    armSetPointTraveling = .29;
    armSetPointScoring = .36;
    elevatorSetPointScoring = (-9.2);
    //setPointTransporting = .25; not used for L3, leave for now
    armP = 1.75;

// Elevator Constants
    elevatorPAccelerate = .03;
    elevatorPConstant = .08;
    //elevatorPPrecise = .1;

// Boolean Flags
    stage1 = true;
    stage2 = false;
    armOn = true;
    dropCheck = true;
    timerStart = true;
    pullback = true;
    pullbackTimerStart = true;
  }

// Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  SmartDashboard.putBoolean("check1 elevator engaged ", check1);
  SmartDashboard.putBoolean("check2 Elevator stage 1", check2);
  SmartDashboard.putBoolean("check3 elevator stage 2", check3);
  SmartDashboard.putBoolean("check4 Timer start", check4);
  SmartDashboard.putBoolean("check5 Shoot coral", check5);
  SmartDashboard.putBoolean("check6 Elevator going down", check6);
  SmartDashboard.putBoolean("check7 Arm back in position", check7);

  //SmartDashboard.putNumber("elevatorEncoderValueCheck", (elevatorEncoderValue/1000));

 // SmartDashboard.putBoolean("Timer Start", dropCheck);
  armEncoderValue = Robot.armEncoder.get();
  elevatorEncoderValue = Robot.elevatorEncoder.get();
  
// Looks to See if the Arm is outside of the acceptable range and if it is the arm PID is set to setPointScoring
    if (armOn == true && (armEncoderValue < .26 || armEncoderValue > .3) && dropCheck == true){
    //System.out.println("Arm PID setPointScoring");

      armEncoderValue = Robot.armEncoder.get();
      armOut = ((armSetPointTraveling - (((armEncoderValue)))) * armP);
       m_ArmSubsystem.m_ArmMotor.set(armOut);
    }

// If the Arm is Within the Acceptable Range then run the elevator
    if (armEncoderValue > .26 && armEncoderValue < .3 && stage1 == true && dropCheck == true){
      if(pullbackTimerStart = true){
      pullbackTime = System.currentTimeMillis();
      pullbackTimerStart = false;
      }
      if(pullback = true){
      if((System.currentTimeMillis() - pullbackTime) <.4){
        m_ArmSubsystem.m_GrabberMotor.set(-.1);
      }
      if((System.currentTimeMillis() - pullbackTime) >=.4){
        m_ArmSubsystem.m_GrabberMotor.set(0);
        pullback = false;
      }
    }
      check1 = true;
    //System.out.println(timerStart);
    //System.out.println("Arm PID setPointScoring");
      armEncoderValue = Robot.armEncoder.get();
  // Arm PID output
     armOut = ((armSetPointTraveling - (((armEncoderValue)))) * armP);
    //  m_ArmSubsystem.m_ArmMotor.set(armOut);
  
  
      elevatorEncoderValue = Robot.elevatorEncoder.get();
       //System.out.println("Arm PID setPointScoring && Elevator Running");

  // If the Elevator is under <= one rotation accelerate the motors 
      if((elevatorEncoderValue/1000) <= 1){
        if(Robot.BMasterStagingSensor.get() == false){
          m_ArmSubsystem.m_GrabberMotor.set(-.1);
         }
         if(Robot.BMasterStagingSensor.get() == true){
           m_ArmSubsystem.m_GrabberMotor.set(0);
         }
    
        check2 = true;
      elevatorOut = (0) + (((elevatorEncoderValue)/1000 + (elevatorSetPointScoring))* elevatorPAccelerate); // Leave for Legacy purposes ((0.000001 * (((-elevatorEncoderValue)/1000) - ((-elevatorEncoderValueCheck)/1000))) / ((System.currentTimeMillis()-engagetime)/1000));
      m_ElevatorSubsystem.elevatorMotors(-elevatorOut);
       //System.out.println("Arm PID setPointScoring && Elevator PAccelerating");
     }
      if((elevatorEncoderValue/1000) > 1){
        if(Robot.BMasterStagingSensor.get() == false){
          m_ArmSubsystem.m_GrabberMotor.set(-.1);
         }
         if(Robot.BMasterStagingSensor.get() == true){
           m_ArmSubsystem.m_GrabberMotor.set(0);
         }
  
        check3 = true;
        elevatorOut = (0) + (((elevatorEncoderValue)/1000 + (elevatorSetPointScoring))* elevatorPConstant);
        m_ElevatorSubsystem.elevatorMotors(-elevatorOut);
       //System.out.println("Arm Pid setPointScoring && Elevator PConstant");
      }
      //ystem.out.println((elevatorEncoderValueCheck/1000));
      
    }
    if((elevatorEncoderValue/1000) > 7.69 && timerStart == true){
      stage1 = false;
      stage2 = true;
      armOn = false;
      check4 = true;
    }
    if(stage2 == true && (elevatorEncoderValue/1000) > 7.69){
      check5 = true;

      elevatorEncoderValue = Robot.elevatorEncoder.get();
      elevatorOut = (0) + (((elevatorEncoderValue)/1000 + (elevatorSetPointScoring))* elevatorPConstant);
      m_ElevatorSubsystem.elevatorMotors(-elevatorOut);

      if(armOn == false && armEncoderValue > .26 && armEncoderValue < .3){
        armEncoderValue = Robot.armEncoder.get();
        armOut = ((armSetPointScoring - (((armEncoderValue)))) * armP);
         m_ArmSubsystem.m_ArmMotor.set(armOut);
        }// if(armOn == false && (armEncoderValue <= .37 || armEncoderValue >= .41)){
      // armEncoderValue = Robot.armEncoder.get();
      // armOut = ((armSetPointScoring - (((armEncoderValue)))) * armP);
      //  m_ArmSubsystem.m_ArmMotor.set(armOut);
      // }
      if(timerStart == true && armOn == false && armEncoderValue > .33 && armEncoderValue < .37){
         engagetime = System.currentTimeMillis();
         timerStart = false;
        }
      // if(armOn == false && (armEncoderValue > .37 && armEncoderValue < .41) && timerStart == true){
      //  engagetime = System.currentTimeMillis();
      //  timerStart = false;
      // }
      if(timerStart == false && System.currentTimeMillis() - engagetime <300){
        check5 = true;
      m_ArmSubsystem.m_GrabberMotor.set(.4); 
      }
      if(timerStart == false && System.currentTimeMillis() - engagetime >=300){
        armOn = true;
        m_ArmSubsystem.m_GrabberMotor.set(0); 
      }

    }
    if(timerStart == false && System.currentTimeMillis() - engagetime >=300 && armEncoderValue > .26 && armEncoderValue < .3 && stage1 == false){
      check6 = true;
      check5 = false;
      stage2 = false;
      stage1 = false;
        elevatorOut = .01;
        m_ElevatorSubsystem.elevatorMotors(elevatorOut);
    }
    if(stage1 == false && stage2 == false && (elevatorEncoderValue/1000) <= .3){
      armOn = false;
      check7 = true;
      System.out.println("home condition");
      armEncoderValue = Robot.armEncoder.get();
 
       armOut = ((armSetPointIntaking - (((armEncoderValue)))) * armP); //(D * ((armEncoderValueBottom)-(armEncoderValueTop))/(timeBottom - timeTop));
       m_ArmSubsystem.m_ArmMotor.set(armOut);

       elevatorOut = 0;
       m_ElevatorSubsystem.elevatorMotors(elevatorOut);
    }
    
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ElevatorSubsystem.elevatorMotors(0);
    m_ArmSubsystem.m_ArmMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
