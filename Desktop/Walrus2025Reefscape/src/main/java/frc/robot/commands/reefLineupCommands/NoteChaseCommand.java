// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.reefLineupCommands;

import java.util.function.DoublePredicate;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class NoteChaseCommand extends Command {

  CommandSwerveDrivetrain m_CommandSwerveDrivetrain;
 // ShootingSubsystem m_ShootingSubsystem;

  private Timer Runtime;
  private double rotSpeed;
  private double yDist;
  private double ATag;
  private double MaxSpeed = 1;
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private double area;
  private double rotation;
  private double engagetime;
  private boolean driveCheck;


  private SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      // .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
      .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

  /** Creates a new NoteOffsetCommand. */
  public NoteChaseCommand(CommandSwerveDrivetrain commandSwerveDrivetrain, double Timeout, double ATag) {

    m_CommandSwerveDrivetrain = commandSwerveDrivetrain;
    addRequirements(m_CommandSwerveDrivetrain);
    // m_ShootingSubsystem = shootingSubsystem;
    // addRequirements(m_ShootingSubsystem);
    Runtime = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveCheck = true;
    Runtime.reset();
    Runtime.start();
    //m_IntakeSubsystem.setspeed(1.0);

    System.out.println("Note Aim Started Started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     // double tx = LimelightHelpers.getTX("limelight-note");

    // boolean RunNote = LimelightHelpers.getTV("limelight-note");

    ATag  = SmartDashboard.getNumber("ATag Detector", 0);
    rotSpeed = SmartDashboard.getNumber("Limelight", 0);
    area = SmartDashboard.getNumber("ATag Area", ATag);
    rotation = SmartDashboard.getNumber("rotation", 0);
    yDist = (rotSpeed);

    

    // RoboPose = m_CommandSwerveDrivetrain.getrobotpose();
    if (ATag == 1){

      System.out.println("ATag 1");

    m_CommandSwerveDrivetrain.setControl( robotCentric
    // .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage)
    .withVelocityX(1/(area) * 4) //rotSpeed*-.02 java.lang.Math.sqrt
    .withVelocityY(-yDist*.04)//
    .withDeadband(0)

    .withRotationalRate(rotation*.08));
  }
  if( ATag == 0 && driveCheck == true) {
    engagetime = System.currentTimeMillis();
    System.out.println("Clock Start");
    driveCheck = false;
  }
  if( ATag == 0 && driveCheck == false && (System.currentTimeMillis() - engagetime) < 300){
    System.out.println("Driving forward");


    System.out.println((System.currentTimeMillis() - engagetime));
    m_CommandSwerveDrivetrain.setControl( robotCentric
    // .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage)
    .withVelocityX( 0.5)
    .withDeadband(0)
    .withVelocityY(0)
    
    
    .withRotationalRate(0)); 
     }
    if (ATag == 0 && driveCheck == false && (System.currentTimeMillis()-engagetime) > 300){
      System.out.println("done");
      m_CommandSwerveDrivetrain.setControl( robotCentric
      // .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage)
      .withVelocityX( 0)
      .withDeadband(0)
      .withVelocityY(0)
      
      
      .withRotationalRate(0)); 
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
