 /************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot;

import com.stuypulse.robot.commands.SeedFieldRelative;
import com.stuypulse.robot.commands.auton.misc.DoNothingAuton;
import com.stuypulse.robot.commands.elevator.ElevatorToFeed;
import com.stuypulse.robot.commands.elevator.ElevatorToLvl2;
import com.stuypulse.robot.commands.elevator.ElevatorToLvl3;
import com.stuypulse.robot.commands.elevator.ElevatorToLvl4;
import com.stuypulse.robot.commands.elevator.ElevatorWaitUntilAtTargetHeight;
import com.stuypulse.robot.commands.shooter.ShooterShoot;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveDriveAlignedToNearestCoralStation;
import com.stuypulse.robot.commands.swerve.SwerveDrivePIDToNearestBranch;
import com.stuypulse.robot.commands.swerve.SwerveDrivePIDToPose;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.funnel.Funnel;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.LimelightVision;
import com.stuypulse.robot.util.PathUtil.AutonConfig;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);
    
    // Subsystems
    public final Elevator elevator = Elevator.getInstance();
    public final SwerveDrive swerve = SwerveDrive.getInstance();
    public final LimelightVision vision = LimelightVision.getInstance();
    public final Odometry odometry = Odometry.getInstance();
    public final Shooter shooter = Shooter.getInstance();
    public final Funnel funnel = Funnel.getInstance();
    public final SimpleSubsystem simple_subsystem = SimpleSubsystem.getInstance();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot Container
    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();
    }

    // ROOKIES, YOUR WORK SHOULD BE INSIDE THE configureAutons() FUNCTION BELOW!
    public void configureAutons() {
        // DON'T CHANGE BELOW 3 LINES
        swerve.configureAutoBuilder();
        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());
        // DON'T CHANGE ABOVE 3 LINES

        /*
         * You can add more autons by creating AutonConfig objects.
         * Each AutonConfig contains a name, command, and a list of path filenames.
         * For example, the EXAMPLE_AUTO below runs the ExampleAuto() command and uses the two paths, Path 1 and Path 2.
         */
        AutonConfig EXAMPLE_AUTO = new AutonConfig(
            "EXAMPLE AUTO", // This is a string representing the name of the auton on SmartDashboard.
            ExampleAuto::new, // This is an instance of the actual auton command.
            "Example Auto Path 1", "Example Auto Path 2" // Make sure the filenames you pass as arguments to the AutonConfig match the filenames in PathPlanner.
        );
        /*
         * Remember: paths are created using PathPlanner! 
         * To get started making paths, open PathPlanner and select "Open Robot Project" and this directory, "Alpha-Bot-2025".
         */
        EXAMPLE_AUTO.registerBlue(autonChooser); // After you create an AutonConfig, make sure to add it using the registerRed()/registerBlue() functions

        // DON'T TOUCH THE LINE BELOW
        SmartDashboard.putData("Autonomous", autonChooser);
    }


    // Feel free to browse through the code after this point, but don't worry if you don't understand any of it.

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

        driver.getLeftButton().onTrue(new IncrementSimpleTarget(100)); // Z
        driver.getBottomButton().onTrue(new IncrementSimpleTarget(-100)); // X
        driver.getLeftButton();
        driver.getRightButton();

        driver.getDPadUp().onTrue(new SeedFieldRelative());

        driver.getLeftTriggerButton()
            .whileTrue(new ElevatorToLvl4()
                .andThen(new ElevatorWaitUntilAtTargetHeight())
                .andThen(new ShooterShoot())
            )
            .onFalse(new ElevatorToFeed())
            .onFalse(new ShooterStop());

        driver.getLeftBumper()
            .whileTrue(new SwerveDrivePIDToPose(new Pose2d(1, 1, new Rotation2d())));

        driver.getRightTriggerButton()
            .whileTrue(new SwerveDriveDriveAlignedToNearestCoralStation(driver));
        
        driver.getRightBumper()
            .whileTrue(new ShooterShoot())
            .onFalse(new ShooterStop())
            .onFalse(new ElevatorToFeed());
        
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
        
        // driver.getLeftButton().whileTrue(new SwerveDrivePIDToNearestBranch());
    }

    /**************/
    /*** AUTONS ***/
    /**************/

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
