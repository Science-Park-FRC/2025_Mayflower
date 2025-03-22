// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    public final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final RollerSubsystem m_RollerSubsystem = new RollerSubsystem();
    // The driver's controller
    CommandPS5Controller m_driverController = new CommandPS5Controller(OIConstants.kDriverControllerPort);

    // Change to use Commands instead of integers
    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Register named commands for PathPlanner
        registerNamedCommands();
        
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> m_robotDrive.drive(
                                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                                true),
                        m_robotDrive));

        m_RollerSubsystem.setDefaultCommand(new RunCommand(
                () -> m_RollerSubsystem.runMotor((m_driverController.getR2Axis() + 1) / 7.5), m_RollerSubsystem));
        
        // Create auto chooser
        createAutoChooser();
    }

    /**
     * Register named commands used in PathPlanner autos
     */
    private void registerNamedCommands() {
        // Register named commands for PathPlanner autos
        System.out.println("SKIBIDI!!! PathPlanner named commands registered");
        
        NamedCommands.registerCommand("shooting coral", 
            new InstantCommand(() -> m_RollerSubsystem.runMotor(0.5), m_RollerSubsystem)
            .andThen(new WaitCommand(3))
            .andThen(new InstantCommand(() -> m_RollerSubsystem.runMotor(0), m_RollerSubsystem)));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        m_driverController.cross()
                .whileTrue(new RunCommand(
                        () -> m_robotDrive.setX(),
                        m_robotDrive));
        m_driverController.square()
                .onTrue(m_robotDrive.zeroYaw());
    }

    /**
     * Create auto chooser with all PathPlanner autos
     */
    private void createAutoChooser() {
        // Create empty auto chooser
        autoChooser = new SendableChooser<>();
        System.out.println("SKIBIDI!!! PathPlanner autos loaded");
        
        // Explicitly add each auto with alliance mirroring
        
        // CORAL AUTO L
        Command coralAutoL = new PathPlannerAuto(
            "Coral Auto (L)", 
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
        );
        autoChooser.setDefaultOption("Coral Auto (L)", coralAutoL);
        
        // CORAL AUTO M
        Command coralAutoM = new PathPlannerAuto(
            "Coral Auto (M)", 
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
        );
        autoChooser.addOption("Coral Auto (M)", coralAutoM);
        
        // CORAL AUTO R
        Command coralAutoR = new PathPlannerAuto(
            "Coral Auto (R)", 
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
        );
        autoChooser.addOption("Coral Auto (R)", coralAutoR);
        
        // Add legacy auto options
        autoChooser.addOption("Leave + Score", 
            new InstantCommand(() -> m_robotDrive.drive(0.25, 0, 0, false), m_robotDrive)
            .andThen(new WaitCommand(4))
            .andThen(new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false), m_robotDrive))
            .andThen(new InstantCommand(() -> m_RollerSubsystem.runMotor(0.5), m_RollerSubsystem))
            .andThen(new WaitCommand(3))
            .andThen(new InstantCommand(() -> m_RollerSubsystem.runMotor(0), m_RollerSubsystem)));
            
        autoChooser.addOption("Leave", 
            new InstantCommand(() -> m_robotDrive.drive(0.25, 0, 0, false), m_robotDrive)
            .andThen(new WaitCommand(2))
            .andThen(new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false), m_robotDrive)));
        
        // Put the chooser on the dashboard
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Just return the selected command from the chooser
        return autoChooser.getSelected();
    }
}