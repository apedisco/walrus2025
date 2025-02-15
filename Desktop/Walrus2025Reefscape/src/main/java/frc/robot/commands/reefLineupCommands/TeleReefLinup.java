// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.reefLineupCommands;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.PointWheelsAt;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleReefLinup extends Command {

  CommandSwerveDrivetrain m_CommandSwerveDrivetrain;

  private double xSpeed;
  private double yDist;
  private double ATag;
  private double MaxSpeed = 1;
  private double MaxAngularRate = 1.5 * Math.PI;
  private boolean pathEnabled;
  private double engagetime;

  private SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      // .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
      .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);
  
      /** Creates a new TeleReefLinup. */
  public TeleReefLinup(CommandSwerveDrivetrain commandSwerveDrivetrain, double Timeout, double ATag) {

     m_CommandSwerveDrivetrain = commandSwerveDrivetrain;
     addRequirements(m_CommandSwerveDrivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    engagetime = System.currentTimeMillis();
    ATag = 0;
    pathEnabled = false;
    System.out.println("ReedLinup Int");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ATag  = SmartDashboard.getNumber("ATag Detector", 0);
    xSpeed = SmartDashboard.getNumber("Limelight", 0);

    if ((System.currentTimeMillis()-engagetime) < 300){
      System.out.println(ATag);
    m_CommandSwerveDrivetrain.setControl( robotCentric
    // .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage)
    .withVelocityX(0.4)
    .withVelocityY(0)
    .withDeadband(0)
    .withRotationalRate(0));//-xSpeed* .1
    
    pathEnabled = true;
  }
  
  else {
    m_CommandSwerveDrivetrain.setControl( robotCentric
    // .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage)
    .withVelocityX( 0)
    .withDeadband(0)
    .withVelocityY(0)
    .withRotationalRate(0)); 

    pathEnabled = false;
    }
  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
