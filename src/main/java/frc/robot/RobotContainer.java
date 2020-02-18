/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
//import frc.robot.commands.Align_PID;
import frc.robot.commands.Arcade_Drive;
import frc.robot.commands.Climb_Down;
import frc.robot.commands.Climb_Up;
import frc.robot.commands.Compressor_Start;
//import frc.robot.commands.Find_Target;
import frc.robot.commands.Intake;
import frc.robot.commands.Intake_Stop;
import frc.robot.commands.Move_Hook;
import frc.robot.commands.Piston_Intake_In;
import frc.robot.commands.Piston_Intake_Out;
import frc.robot.commands.Set_LED;
import frc.robot.commands.Shooter_Shoot_Button;
//import frc.robot.commands.Shooter_Shoot_Joy;
import frc.robot.commands.Stop_Climb;
import frc.robot.commands.Stop_Shoot;
import frc.robot.subsystems.Climber_Subsystem;
import frc.robot.subsystems.Compressor_Subsystem;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.DriveTrain_Subsystem;
//import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.NavXIMU_Subsystem;
import frc.robot.subsystems.Shooter_Subsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.controller.PIDController;


import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;

//import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    // Subsystems //

    private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
    private final Climber_Subsystem m_climberSubsystem = new Climber_Subsystem();
    private final Shooter_Subsystem m_shooterSubsystem = new Shooter_Subsystem();
    private final Intake_Subsystem m_intakeSubsystem = new Intake_Subsystem();
    private final Limelight_Subsystem n_limelightSubsystem = new Limelight_Subsystem();
    private final Compressor_Subsystem p_compressorSubsystem = new Compressor_Subsystem();
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    //private final Drivetrain m_drive = new Drivetrain();

    // Commands //
    private final Arcade_Drive m_arcadeDrive = new Arcade_Drive(m_driveSubsystem);
    private final Move_Hook m_Move_Hook = new Move_Hook(m_climberSubsystem);
    private final Climb_Up m_climbUpCommand = new Climb_Up(m_climberSubsystem);
    private final Climb_Down m_climbDownCommand = new Climb_Down(m_climberSubsystem);
    private final Stop_Climb m_stopClimbCommand = new Stop_Climb(m_climberSubsystem);
    //private final Shooter_Shoot_Joy m_Shooter_Shoot_Joy = new Shooter_Shoot_Joy(m_shooterSubsystem);
    private final Shooter_Shoot_Button m_Shooter_Shoot_Button = new Shooter_Shoot_Button(m_shooterSubsystem);
    private final Stop_Shoot m_Stop_Shoot = new Stop_Shoot(m_shooterSubsystem);
    private final Intake m_Intake = new Intake(m_intakeSubsystem);
    private final Intake_Stop m_Intake_Stop = new Intake_Stop(m_intakeSubsystem);
    private final Piston_Intake_Out p_Intake_Out = new Piston_Intake_Out(m_intakeSubsystem);
    private final Piston_Intake_In p_Intake_In = new Piston_Intake_In(m_intakeSubsystem);
    //private final Find_Target n_Find_Target = new Find_Target(n_limelightSubsystem, m_driveSubsystem);
    private final Set_LED n_Set_LED = new Set_LED(n_limelightSubsystem);
    //private final Align_PID n_Align_PID = new Align_PID(m_driveSubsystem, n_limelightSubsystem);
    private final Compressor_Start p_Compressor_Start = new Compressor_Start(p_compressorSubsystem);


    // Commands for Autonomous Period //
    //private final Arcade_Drive m_autoCommand = new Arcade_Drive(m_driveTrainSubsystem);

    // Controller Mappings //

    /* Robot Driver - AKA Driver 1 */
    public static final Joystick j_stick_driver = new Joystick(0);  // Drive joystick (0) initialization
    public static final JoystickButton j_stick_driver_LB = new JoystickButton(j_stick_driver, Constants.BUTTON_RIGHT);              // Left button
    public static final JoystickButton j_stick_driver_RB = new JoystickButton(j_stick_driver, Constants.BUTTON_LEFT);               // Right button 
    public static final JoystickButton j_stick_driver_X = new JoystickButton(j_stick_driver, Constants.BUTTON_X);                   // X button
    public static final JoystickButton j_stick_driver_Y = new JoystickButton(j_stick_driver, Constants.BUTTON_Y);                   // Y button
    public static final JoystickButton j_stick_driver_A = new JoystickButton(j_stick_driver, Constants.BUTTON_A);                   // A button
    public static final JoystickButton j_stick_driver_B = new JoystickButton(j_stick_driver, Constants.BUTTON_B);                   // B button
    public static final JoystickButton j_stick_driver_Back = new JoystickButton(j_stick_driver, Constants.BUTTON_BACK);             // Back button
    public static final JoystickButton j_stick_driver_Start = new JoystickButton(j_stick_driver, Constants.BUTTON_START);           // Start button
    public static final JoystickButton j_stick_driver_leftStick = new JoystickButton(j_stick_driver, Constants.BUTTON_LEFTSTICK);   // Left-Stick button
    public static final JoystickButton j_stick_driver_rightStick = new JoystickButton(j_stick_driver, Constants.BUTTON_RIGHTSTICK); // Right-Stick button

    /* Robot Controller - AKA Driver 2 */
    public static final Joystick j_stick_control = new Joystick(1);  // Control joystick (1) initialization
    public static final JoystickButton j_stick_control_LB = new JoystickButton(j_stick_control, Constants.BUTTON_RIGHT);              // Left button
    public static final JoystickButton j_stick_control_RB = new JoystickButton(j_stick_control, Constants.BUTTON_LEFT);               // Right button 
    public static final JoystickButton j_stick_control_X = new JoystickButton(j_stick_control, Constants.BUTTON_X);                   // X button
    public static final JoystickButton j_stick_control_Y = new JoystickButton(j_stick_control, Constants.BUTTON_Y);                   // Y button
    public static final JoystickButton j_stick_control_A = new JoystickButton(j_stick_control, Constants.BUTTON_A);                   // A button
    public static final JoystickButton j_stick_control_B = new JoystickButton(j_stick_control, Constants.BUTTON_B);                   // B button
    public static final JoystickButton j_stick_control_Back = new JoystickButton(j_stick_control, Constants.BUTTON_BACK);             // Back button
    public static final JoystickButton j_stick_control_Start = new JoystickButton(j_stick_control, Constants.BUTTON_START);           // Start button
    public static final JoystickButton j_stick_control_leftStick = new JoystickButton(j_stick_control, Constants.BUTTON_LEFTSTICK);   // Left-Stick button
    public static final JoystickButton j_stick_control_rightStick = new JoystickButton(j_stick_control, Constants.BUTTON_RIGHTSTICK); // Right-Stick button

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings //
        configureButtonBindings();
        
        // Default Command(s) //
        m_driveSubsystem.setDefaultCommand(m_arcadeDrive);  // Defaults to Arcade Drive
        m_climberSubsystem.setDefaultCommand(m_stopClimbCommand);       // Defaults to climber not running
        m_climberSubsystem.setDefaultCommand(m_Move_Hook);       // Defaults to climber not running
        m_shooterSubsystem.setDefaultCommand(m_Stop_Shoot);
        m_intakeSubsystem.setDefaultCommand(m_Intake_Stop);
        p_compressorSubsystem.setDefaultCommand(p_Compressor_Start);
        n_limelightSubsystem.setDefaultCommand(n_Set_LED);
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        //j_stick_control_A.whenHeld(m_climbUpCommand);
        //j_stick_control_A.whenReleased(m_stopClimbCommand);

        //j_stick_control_B.whenHeld(m_Intake);
        //j_stick_control_B.whenReleased(m_Intake_Stop);

        j_stick_control_X.whenPressed(p_Intake_Out);
        j_stick_control_Y.whenPressed(p_Intake_In);

        j_stick_driver_A.whenHeld(m_Shooter_Shoot_Button);
        j_stick_driver_A.whenReleased(m_Stop_Shoot);

        //j_stick_driver_B.whenHeld(n_Align_PID);
        //j_stick_driver_B.whenReleased(n_Set_LED);
    }






    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public Command getAutonomousCommand() {


         // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        10);

// Create config for trajectory
TrajectoryConfig config =
    new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

// An example trajectory to follow.  All units in meters.
Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(
        new Translation2d(1, 1),
        new Translation2d(2, -1)
    ),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(3, 0, new Rotation2d(0)),
    // Pass config
    config
);

RamseteCommand ramseteCommand = new RamseteCommand(
    exampleTrajectory,
    m_robotDrive::getPose,
    new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    new SimpleMotorFeedforward(DriveConstants.ksVolts,
                               DriveConstants.kvVoltSecondsPerMeter,
                               DriveConstants.kaVoltSecondsSquaredPerMeter),
    DriveConstants.kDriveKinematics,
    m_robotDrive::getWheelSpeeds,
    new PIDController(DriveConstants.kPDriveVel, 0, 0),
    new PIDController(DriveConstants.kPDriveVel, 0, 0),
    // RamseteCommand passes volts to the callback
    m_robotDrive::tankDriveVolts,
    m_robotDrive
);

// Run path following command, then stop at the end.
return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
}
        // An ArcadeDriveCommand will run in autonomous
        //return m_autoCommand;
/*
        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2.0), Units.feetToMeters(2.0));
    config.setKinematics(m_drive.getKinematics());

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        Arrays.asList(new Pose2d(), new Pose2d(1.0, 0, new Rotation2d()),
            new Pose2d(2.3, 1.2, Rotation2d.fromDegrees(90.0))),
        config
    );

    RamseteCommand command = new RamseteCommand(
        trajectory,
        m_drive::getPose,
        new RamseteController(2, .7),
        m_drive.getFeedforward(),
        m_drive.getKinematics(),
        m_drive::getSpeeds,
        m_drive.getLeftPIDController(),
        m_drive.getRightPIDController(),
        m_drive::setOutputVolts,
        m_drive
    );

    return command.andThen(() -> m_drive.setOutputVolts(0, 0));
  }

  public void reset() {
    m_drive.reset();
    }

*/

    
}
