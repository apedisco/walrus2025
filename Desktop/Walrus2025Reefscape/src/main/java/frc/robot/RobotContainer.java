// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.PointWheelsAt;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.path.PathPlannerPath;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.autoCommands.IntakeCommandAuto;
import frc.robot.autoCommands.L2ScoreCommandAuto;
import frc.robot.autoCommands.L3ScoreCommandAuto;
import frc.robot.autoCommands.L4ScoreCommandAuto;
import frc.robot.autoCommands.reefLineupRightCommandAuto;
import frc.robot.autoCommands.reefLinupLeftCommandAuto;
import frc.robot.commands.intakeDropCommand;
import frc.robot.commands.L_ScoreCommands.L1ScoreCommand;
import frc.robot.commands.L_ScoreCommands.L2ScoreCommand;
import frc.robot.commands.L_ScoreCommands.L3ScoreCommand;
import frc.robot.commands.L_ScoreCommands.L4Drop;
import frc.robot.commands.L_ScoreCommands.L4Raise;
import frc.robot.commands.L_ScoreCommands.L4ScoreCommand;
import frc.robot.commands.L_ScoreCommands.L4Shoot;
import frc.robot.commands.L_ScoreCommands.Test1Command;
import frc.robot.commands.L_ScoreCommands.Test2Command;
import frc.robot.commands.TestCommands.armTestCommand;
import frc.robot.commands.TestCommands.armTestCommand2;
import frc.robot.commands.TestCommands.creepModeHold;
import frc.robot.commands.TestCommands.elevatorCommandTest;
import frc.robot.commands.TestCommands.theLebron;
import frc.robot.commands.armPIDCommands.armPIDIntakeCommand;
import frc.robot.commands.armPIDCommands.armPIDCommandBall;
import frc.robot.commands.armPIDCommands.armPIDCommandBallBack;
import frc.robot.commands.armPIDCommands.armPIDCommandBallGround;
import frc.robot.commands.armPIDCommands.armPIDCommandBallShoot;
import frc.robot.commands.armPIDCommands.ballL3Command;
import frc.robot.commands.buttonPressOutputCommands.panel1ButtonOutput;
import frc.robot.commands.reefLineupCommands.reefLineupLeftCommand;
import frc.robot.commands.reefLineupCommands.reeflineupPigeonCommand;
import frc.robot.commands.reefLineupCommands.reefLineupRightCommand;
import frc.robot.commands.reefLineupCommands.reefLinupLeftCommandTEST;
import frc.robot.commands.reefLineupCommands.reefLinupRightCommandTEST;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsytem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {

    // private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); //test to set creep mode




    private double MaxAngularRate = RotationsPerSecond.of(.4).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity .75

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    SendableChooser<Command> m_chooser = new SendableChooser<>();

    private final CommandJoystick m_DriveJoystick = new CommandJoystick(0);
    private final Joystick m_ElevatorJoystick = new Joystick(1);
    // private final Joystick m_TestJoystick = new Joystick(2);
    private final Joystick m_ButtonPanel = new Joystick(3);

    public final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
    public final ClimbSubsytem m_ClimbSubsytem = new ClimbSubsytem();
    

    public RobotContainer() {
        NamedCommands.registerCommand("L2ScoreCommand", new L2ScoreCommandAuto(m_ArmSubsystem).withTimeout(6));
        NamedCommands.registerCommand("L4ScoreCommand", new L4ScoreCommandAuto(m_ArmSubsystem, m_ElevatorSubsystem).withTimeout(8));
        NamedCommands.registerCommand("reefLineupRightCommand", new reefLineupRightCommandAuto(drivetrain, MaxAngularRate, MaxAngularRate, MaxSpeed, MaxAngularRate).withTimeout(8));
        NamedCommands.registerCommand("reefLineupLeftCommand", new reefLinupLeftCommandAuto(drivetrain, MaxAngularRate, MaxAngularRate, MaxSpeed, MaxAngularRate).withTimeout(8));
        NamedCommands.registerCommand("IntakeCommand", new IntakeCommandAuto(m_ArmSubsystem, m_ElevatorJoystick, this.getAutonomousCommand()));

        SmartDashboard.putData("Autonomous", m_chooser);
     m_chooser.setDefaultOption("Nothing", new PathPlannerAuto("Nothing"));
     m_chooser.addOption("LSideCD_L4", new PathPlannerAuto("LSideCD_L4"));
     m_chooser.addOption("Center_Start", new PathPlannerAuto("Center_Start"));
     m_chooser.addOption("Left_Side_Start", new PathPlannerAuto("Left_Side_Start"));

     
   
         
        configureBindings();
        //Creates the Buttons for the Elevator Joystick%
        //  final JoystickButton elevator1Button = new JoystickButton(m_ElevatorJoystick, 1);

        // Creates the Button for the Test Joystick
        //  final JoystickButton test1Button = new JoystickButton(m_TestJoystick, 1);
        //  final JoystickButton test2Button = new JoystickButton(m_TestJoystick, 2);
        //  final JoystickButton test3Button = new JoystickButton(m_TestJoystick, 3);
        //  final JoystickButton test4Button = new JoystickButton(m_TestJoystick, 4);
        //  final JoystickButton test5Button = new JoystickButton(m_TestJoystick, 5);
        //  final JoystickButton test6Button = new JoystickButton(m_TestJoystick, 6);
        //  final JoystickButton test7Button = new JoystickButton(m_TestJoystick, 7);
        //  final JoystickButton test8Button = new JoystickButton(m_TestJoystick, 8);
        //  final JoystickButton test9Button = new JoystickButton(m_TestJoystick, 9);
        //  final JoystickButton test10Button = new JoystickButton(m_TestJoystick, 10);
        //  final JoystickButton test11Button = new JoystickButton(m_TestJoystick, 11);

         //Creates the Buttons for the Button Panel
         final JoystickButton panel1Button = new JoystickButton(m_ButtonPanel, 1);
         final JoystickButton panel2Button = new JoystickButton(m_ButtonPanel, 2);
         final JoystickButton panel3Button = new JoystickButton(m_ButtonPanel, 3);
         final JoystickButton panel4Button = new JoystickButton(m_ButtonPanel, 4);
         final JoystickButton panel5Button = new JoystickButton(m_ButtonPanel, 5);
         final JoystickButton panel6Button = new JoystickButton(m_ButtonPanel, 6);
         final JoystickButton panel7Button = new JoystickButton(m_ButtonPanel, 7);
         final JoystickButton panel8Button = new JoystickButton(m_ButtonPanel, 8);
         final JoystickButton panel9Button = new JoystickButton(m_ButtonPanel, 9);
         final JoystickButton panel10Button = new JoystickButton(m_ButtonPanel, 10);
         final JoystickButton panel11Button = new JoystickButton(m_ButtonPanel, 11);
        // Tells the m_TestJoystick Buttons What to Do.
        //  test1Button.toggleOnTrue(new armPIDCommandBallGround(m_ArmSubsystem));
        //  test2Button.toggleOnTrue(new ballL3Command(m_ArmSubsystem, m_ElevatorSubsystem));
    //     test1Button.toggleOnTrue(new elevatorCommandTest(m_ElevatorSubsystem, MaxAngularRate, m_ElevatorJoystick, m_TestJoystick));
    //     test3Button.toggleOnTrue(new armTestCommand(m_ArmSubsystem, m_ElevatorJoystick, m_TestJoystick));
    //     test4Button.toggleOnTrue(new armTestCommand2(m_ArmSubsystem, m_ElevatorJoystick, m_TestJoystick));
    //    // test7Button.onTrue(new L3ScoreCommand(m_ArmSubsystem, m_ElevatorSubsystem));
    //     test8Button.toggleOnTrue(new armPIDCommandBall(m_ArmSubsystem));
    //     test9Button.toggleOnTrue(new armPIDCommandBallBack(m_ArmSubsystem));
    //     test10Button.toggleOnTrue(new armPIDCommandBallShoot(m_ArmSubsystem));

      
        // Tells the Panel Butons What to Do.
      panel1Button.toggleOnTrue(new armPIDIntakeCommand(m_ArmSubsystem, m_ElevatorJoystick));
       panel2Button.toggleOnTrue(new Test1Command(m_ArmSubsystem, m_ElevatorSubsystem));
       panel3Button.toggleOnTrue(new Test2Command(m_ArmSubsystem, m_ElevatorSubsystem));
        // panel2Button.whileTrue(new armTestCommand(m_ArmSubsystem, m_ElevatorJoystick, m_ElevatorJoystick, m_ClimbSubsytem));
        // panel3Button.whileTrue(new armTestCommand2(m_ArmSubsystem, m_ElevatorJoystick, m_ElevatorJoystick, m_ClimbSubsytem));
    //    panel3Button.toggleOnTrue(new theLebron(m_ArmSubsystem, m_ElevatorSubsystem));
        panel4Button.toggleOnTrue(new L4ScoreCommand(m_ArmSubsystem, m_ElevatorSubsystem));

        panel5Button.toggleOnTrue(new L3ScoreCommand(m_ArmSubsystem, m_ElevatorSubsystem));
        panel6Button.toggleOnTrue(new L2ScoreCommand(m_ArmSubsystem));
        panel7Button.toggleOnTrue(new L1ScoreCommand(m_ArmSubsystem));
        panel8Button.toggleOnTrue(new armPIDCommandBallShoot(m_ArmSubsystem)); 
        panel9Button.toggleOnTrue(new ballL3Command(m_ArmSubsystem, m_ElevatorSubsystem));
        panel10Button.toggleOnTrue(new armPIDCommandBallBack(m_ArmSubsystem));
        panel11Button.toggleOnTrue(new armPIDCommandBall(m_ArmSubsystem));
        // panel11Button.toggleOnTrue(new creepModeHold());

        // panel2Button.toggleOnTrue(new Test1Command(m_ArmSubsystem, m_ElevatorSubsystem));
        // panel3Button.toggleOnTrue(new Test2Command(m_ArmSubsystem, m_ElevatorSubsystem));
        // panel4Button.toggleOnTrue(new L4Drop(m_ArmSubsystem, m_ElevatorSubsystem));




        // panel2Button.toggleOnTrue(new armPIDCommandBall(m_ArmSubsystem));
        // panel3Button.toggleOnTrue(new armPIDCommandBallBack(m_ArmSubsystem));
        // panel4Button.toggleOnTrue(new L4ScoreCommand(m_ArmSubsystem, m_ElevatorSubsystem));
        // panel5Button.toggleOnTrue(new L3ScoreCommand(m_ArmSubsystem, m_ElevatorSubsystem));
        // panel6Button.toggleOnTrue(new L2ScoreCommand(m_ArmSubsystem));
        // panel7Button.toggleOnTrue(new L1ScoreCommand(m_ArmSubsystem));
        // panel8Button.toggleOnTrue(new armPIDCommandBallShoot(m_ArmSubsystem));
        // panel9Button.toggleOnTrue(new ballL3Command(m_ArmSubsystem, m_ElevatorSubsystem));
        // panel10Button.toggleOnTrue(new armPIDCommandBallBack(m_ArmSubsystem));
        // panel11Button.toggleOnTrue(new armPIDCommandBall(m_ArmSubsystem));
        
        
        // panel1Button.whileTrue(new panel1ButtonOutput());
        // panel1Button.and(m_DriveJoystick.button(2)).toggleOnTrue(new armPIDIntakeCommand(m_ArmSubsystem, m_ElevatorJoystick));
        // panel2Button.toggleOnTrue(new armPIDCommandBall(m_ArmSubsystem));
        // panel3Button.toggleOnTrue(new armPIDCommandBallBack(m_ArmSubsystem));
        // panel4Button.and(m_DriveJoystick.button(2)).toggleOnTrue(new L4ScoreCommand(m_ArmSubsystem, m_ElevatorSubsystem));
        // panel5Button.and(m_DriveJoystick.button(2)).toggleOnTrue(new L3ScoreCommand(m_ArmSubsystem, m_ElevatorSubsystem));
        // panel6Button.and(m_DriveJoystick.button(2)).toggleOnTrue(new L2ScoreCommand(m_ArmSubsystem));
        // panel7Button.and(m_DriveJoystick.button(2)).toggleOnTrue(new L1ScoreCommand(m_ArmSubsystem));
        // panel8Button.and(m_DriveJoystick.button(2)).toggleOnTrue(new armPIDCommandBallShoot(m_ArmSubsystem));
        // panel9Button.and(m_DriveJoystick.button(2)).toggleOnTrue(new ballL3Command(m_ArmSubsystem, m_ElevatorSubsystem));
        // panel10Button.and(m_DriveJoystick.button(2)).toggleOnTrue(new armPIDCommandBallBack(m_ArmSubsystem));
        // panel11Button.and(m_DriveJoystick.button(2)).toggleOnTrue(new armPIDCommandBall(m_ArmSubsystem));
        
    }
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-m_DriveJoystick.getY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-m_DriveJoystick.getX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-m_DriveJoystick.getTwist() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
    
        m_DriveJoystick.button(1).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); // Sets the Forward Direction on the Joystick to whatever the forward direction of the Pigion is currently facing
        m_DriveJoystick.button(3).whileTrue(new reefLineupLeftCommand(drivetrain, MaxAngularRate, MaxAngularRate, MaxAngularRate, MaxSpeed, MaxAngularRate));
        m_DriveJoystick.button(4).whileTrue(new reefLineupRightCommand(drivetrain, MaxAngularRate, MaxAngularRate, MaxSpeed, MaxAngularRate));
        m_DriveJoystick.button(5).whileTrue(new armTestCommand(m_ArmSubsystem, m_ElevatorJoystick, m_ElevatorJoystick, m_ClimbSubsytem));
        m_DriveJoystick.button(6).whileTrue(new armTestCommand2(m_ArmSubsystem, m_ElevatorJoystick, m_ElevatorJoystick, m_ClimbSubsytem));
        m_DriveJoystick.button(8).whileTrue(new intakeDropCommand(m_ClimbSubsytem));
        
        // m_DriveJoystick.button(4).whileTrue(new elevatorCommandTest(m_ElevatorSubsystem, MaxAngularRate, m_ElevatorJoystick, m_TestJoystick));
        // m_DriveJoystick.button(8).whileTrue(new reeflineupPigeonCommand(drivetrain, MaxAngularRate, MaxAngularRate, MaxSpeed, MaxAngularRate));

        // m_DriveJoystick.button(2).whileTrue(drivetrain.applyRequest(() -> brake));
        // m_DriveJoystick.button(6).whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(90))
        // ));
    
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // m_DriveJoystick.button(4).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // m_DriveJoystick.button(5).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // m_DriveJoystick.button(6).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // m_DriveJoystick.button(7).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        
        
    
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // return new L4ScoreCommandAuto(m_ArmSubsystem, m_ElevatorSubsystem);
       return m_chooser.getSelected();
       // return new PathPlannerAuto("Auto");
        //return new reefLineupRightCommandAuto(drivetrain, MaxAngularRate, MaxAngularRate, MaxSpeed, MaxAngularRate);
    }
}
