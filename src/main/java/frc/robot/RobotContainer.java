// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.controller.PIDController;
//import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
//import edu.wpi.first.wpilibj.geometry.Pose2d;
//import edu.wpi.first.wpilibj.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.geometry.Translation2d;
//import edu.wpi.first.wpilibj.trajectory.Trajectory;
//import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
//import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
//import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
//import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private final SwerveSubsystem robotContainerSwerveSubsystem = new SwerveSubsystem();

    private final Joystick driverJoystick = new Joystick(Constants.OIConstants.kDriverControllerPort);

    public RobotContainer() {
        robotContainerSwerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                robotContainerSwerveSubsystem,
                () -> driverJoystick.getRawAxis(Constants.OIConstants.kDriverXAxis),
                () -> driverJoystick.getRawAxis(Constants.OIConstants.kDriverYAxis),
                () -> driverJoystick.getRawAxis(Constants.OIConstants.kDriverRotAxis),
                () -> driverJoystick.getRawButton(Constants.OIConstants.kDriverFieldOrientedButtonIdx)));
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        JoystickButton zeroHeadingButton = new JoystickButton(driverJoystick, 2);
        zeroHeadingButton.onTrue(new InstantCommand(() -> robotContainerSwerveSubsystem.zeroHeading()));

        POVButton upDpad = new POVButton(driverJoystick, 0);
        upDpad.onTrue(new InstantCommand(()-> robotContainerSwerveSubsystem.lockUp()));

        POVButton rightDpad = new POVButton(driverJoystick, 90);
        rightDpad.onTrue(new InstantCommand(()-> robotContainerSwerveSubsystem.lockRight()));

        POVButton downDpad = new POVButton(driverJoystick, 180);
        downDpad.onTrue(new InstantCommand(()-> robotContainerSwerveSubsystem.lockDown()));

        POVButton leftDpad = new POVButton(driverJoystick, 270);
        leftDpad.onTrue(new InstantCommand(()-> robotContainerSwerveSubsystem.lockLeft()));
    }


        public Command getAutonomousCommand() {
        //traj settings???
        //      TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        //              Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        //              Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        //              .setKinematics(DriveConstants.kDriveKinematics);


        //  //tasty trajectory coords
        //      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        //              //optimization is kinda trash ngl
        //              new Pose2d(0, 0, new Rotation2d(0)),
        //              List.of(
        //                      //waypoints?
        //                      new Translation2d(1,0),
        //                     new Translation2d(1,-1),
        //                     new Translation2d(1,-1)
        //             ),
        //             //final destination...
        //             new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
        //             trajectoryConfig
        //     );

        return null;
    }
}