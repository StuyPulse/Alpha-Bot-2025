 /************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.SeedFieldRelative;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveDriveAlignedToNearestCoralStation;
import com.stuypulse.robot.commands.swerve.SwerveDrivePIDToNearestBranch;
import com.stuypulse.robot.commands.swerve.SwerveDrivePIDToPose;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.PathUtil.AutonConfig;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);
    
    // Subsystems
    public final SwerveDrive swerve = SwerveDrive.getInstance();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot Container

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new SwerveDriveDrive(driver));
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {

        // driver.getDPadUp().onTrue(new SeedFieldRelative());

        // driver.getDPadLeft().onTrue(new FunnelDefaultCommand(driver));

        // driver.getLeftTriggerButton()
        //     // .whileTrue(new ElevatorToLvl4()
        //     //     .andThen(new ElevatorWaitUntilAtTargetHeight())
        //     //     .andThen(new ShooterShoot())
        //     // )
        //     // .onFalse(new ElevatorToFeed())
        //     // .onFalse(new ShooterStop());\[]\[]
        //     .whileTrue(new SwerveDrivePIDToNearestBranch());

        // driver.getLeftBumper()
        //     .whileTrue(new SwerveDrivePIDToPose(new Pose2d(1, 1, new Rotation2d())));

        // driver.getRightTriggerButton()
        //     .whileTrue(new SwerveDriveDriveAlignedToNearestCoralStation(driver));
        
        // driver.getRightBumper()
        //     .whileTrue(new ShooterShoot())
        //     .onFalse(new ShooterStop())
        //     .onFalse(new ElevatorToFeed());
        
        // // Automated L4
        // driver.getTopButton()

        //     .whileTrue(new ElevatorToLvl4()
        //         .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new SwerveDrivePIDToNearestBranch()))
        //         .andThen(new ShooterShoot())
        //     )
        //     .onFalse(new ElevatorToFeed())
        //     .onFalse(new ShooterStop());
        
        // // Automated L3
        // driver.getRightButton()
        //     .whileTrue(new ElevatorToLvl3()
        //         .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new SwerveDrivePIDToNearestBranch()))
        //         .andThen(new ShooterShoot())
        //     )
        //     .onFalse(new ElevatorToFeed())
        //     .onFalse(new ShooterStop());

        // // Automated L2
        // driver.getBottomButton()
        //     .whileTrue(new ElevatorToLvl2()
        //         .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new SwerveDrivePIDToNearestBranch()))
        //         .andThen(new ShooterShoot())
        //     )
        //     .onFalse(new ElevatorToFeed())
        //     .onFalse(new ShooterStop());
        
        // //driver.getRawDPadUp().whileTrue(new FunnelDefaultCommand()

        // driver.getLeftButton().whileTrue(new SwerveDrivePIDToNearestBranch());

        // driver.getLeftMenuButton().whileTrue(new SwerveDrivePIDToPose(new Pose2d(new Translation2d(Field.getClosestBranch().getTargetPose().getX()-1, Field.getClosestBranch().getTargetPose().getY()), Field.getClosestBranch().getTargetPose().getRotation())));
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        swerve.configureAutoBuilder();

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    public static String getAutonomousCommandNameStatic() {
        if (autonChooser.getSelected() == null) {
            return "Do Nothing";
        }

        return autonChooser.getSelected().getName();
    }

}
