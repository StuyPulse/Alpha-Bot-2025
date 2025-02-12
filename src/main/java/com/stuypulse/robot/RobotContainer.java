 /************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.SeedFieldRelative;
import com.stuypulse.robot.commands.auton.EDCB.FourPieceEDCB;
import com.stuypulse.robot.commands.auton.EDCB.OnePieceE;
import com.stuypulse.robot.commands.auton.EDCB.ThreeHalfPieceEDC;
import com.stuypulse.robot.commands.auton.EDCB.ThreePieceEDC;
import com.stuypulse.robot.commands.auton.EDCB.TwoPieceED;
import com.stuypulse.robot.commands.auton.JKLA.FourPieceJKLA;
import com.stuypulse.robot.commands.auton.JKLA.OnePieceJ;
import com.stuypulse.robot.commands.auton.JKLA.ThreeHalfPieceJKL;
import com.stuypulse.robot.commands.auton.JKLA.ThreePieceJKL;
import com.stuypulse.robot.commands.auton.JKLA.TwoPieceJK;
import com.stuypulse.robot.commands.auton.misc.DoNothingAuton;
import com.stuypulse.robot.commands.auton.misc.Mobility;
import com.stuypulse.robot.commands.auton.misc.OnePieceG;
import com.stuypulse.robot.commands.auton.misc.OnePieceH;
import com.stuypulse.robot.commands.auton.tests.CurvyLineTest;
import com.stuypulse.robot.commands.auton.tests.RSquareTest;
import com.stuypulse.robot.commands.auton.tests.SquareTest;
import com.stuypulse.robot.commands.auton.tests.StraightLineTest;
import com.stuypulse.robot.commands.elevator.ElevatorToBottom;
import com.stuypulse.robot.commands.elevator.ElevatorToFeed;
import com.stuypulse.robot.commands.elevator.ElevatorToLvl2;
import com.stuypulse.robot.commands.elevator.ElevatorToLvl3;
import com.stuypulse.robot.commands.elevator.ElevatorToLvl4;
import com.stuypulse.robot.commands.elevator.ElevatorWaitUntilAtTargetHeight;
import com.stuypulse.robot.commands.funnel.FunnelDefaultCommand;
import com.stuypulse.robot.commands.shooter.ShooterAcquire;
import com.stuypulse.robot.commands.shooter.ShooterSetAcquire;
import com.stuypulse.robot.commands.shooter.ShooterShoot;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveDriveAlignedToNearestCoralStation;
import com.stuypulse.robot.commands.swerve.SwerveDrivePIDToNearestBranch;
import com.stuypulse.robot.commands.swerve.SwerveDrivePIDToPose;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.funnel.Funnel;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.LimelightVision;
import com.stuypulse.robot.util.PathUtil.AutonConfig;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

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
    public final Elevator elevator = Elevator.getInstance();
    public final SwerveDrive swerve = SwerveDrive.getInstance();
    public final LimelightVision vision = LimelightVision.getInstance();
    public final Odometry odometry = Odometry.getInstance();
    public final Shooter shooter = Shooter.getInstance();
    public final Funnel funnel = Funnel.getInstance();

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
        funnel.setDefaultCommand(new FunnelDefaultCommand());
        shooter.setDefaultCommand(new ShooterSetAcquire()
            .onlyIf(() -> !shooter.hasCoral() && Math.abs(elevator.getTargetHeight()-Settings.Elevator.FEED_HEIGHT_METERS) < 0.01)
            .andThen(new WaitUntilCommand(() -> shooter.hasCoral()))
            .andThen(new ShooterStop()));
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {

        driver.getDPadUp().onTrue(new SeedFieldRelative());

        driver.getLeftTriggerButton()
            .whileTrue(new ElevatorToLvl4()
                .andThen(new ElevatorWaitUntilAtTargetHeight())
                .andThen(new ShooterShoot())
            )
            .onFalse(new ElevatorToFeed())
            .onFalse(new ShooterStop());

        driver.getRightTriggerButton()
            .whileTrue(new SwerveDriveDriveAlignedToNearestCoralStation(driver));
        
        driver.getRightBumper()
            .whileTrue(new ShooterShoot())
            .onFalse(new ShooterStop())
            .onFalse(new ElevatorToFeed());
        
        // Automated L4
        driver.getTopButton()
            .whileTrue(new ElevatorToLvl4()
                .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new SwerveDrivePIDToNearestBranch()))
                .andThen(new ShooterShoot())
            )
            .onFalse(new ElevatorToFeed())
            .onFalse(new ShooterStop());
        
        // Automated L3
        driver.getRightButton()
            .whileTrue(new ElevatorToLvl3()
                .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new SwerveDrivePIDToNearestBranch()))
                .andThen(new ShooterShoot())
            )
            .onFalse(new ElevatorToFeed())
            .onFalse(new ShooterStop());

        // Automated L2
        driver.getBottomButton()
            .whileTrue(new ElevatorToLvl2()
                .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new SwerveDrivePIDToNearestBranch()))
                .andThen(new ShooterShoot())
            )
            .onFalse(new ElevatorToFeed())
            .onFalse(new ShooterStop());
        
        driver.getLeftButton().whileTrue(new SwerveDrivePIDToNearestBranch());
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        swerve.configureAutoBuilder();

        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());

        /** TOP AUTONS **/

        AutonConfig BLUE_ONE_PIECE_H = new AutonConfig("1 Piece H", OnePieceH::new,
        "Blue Mid Top to H");
        AutonConfig RED_ONE_PIECE_H = new AutonConfig("1 Piece H", OnePieceH::new,
        "Red Mid Top to H");

        AutonConfig BLUE_ONE_PIECE_J = new AutonConfig("1 Piece J", OnePieceJ::new,
        "Blue Top to J");
        AutonConfig RED_ONE_PIECE_J = new AutonConfig("1 Piece J", OnePieceJ::new,
        "Red Top to J");

        AutonConfig BLUE_TWO_PIECE_JK = new AutonConfig("2 Piece JK", TwoPieceJK::new,
        "Blue Top to J", "Blue J to HP", "Blue HP to K");
        AutonConfig RED_TWO_PIECE_JK = new AutonConfig("2 Piece JK", TwoPieceJK::new,
        "Red Top to J", "Red J to HP", "Red HP to K");

        AutonConfig BLUE_THREE_PIECE_JKL = new AutonConfig("3 Piece JKL", ThreePieceJKL::new,
        "Blue Top to J", "Blue J to HP", "Blue HP to K", "Blue K to HP", "Blue HP to L");
        AutonConfig RED_THREE_PIECE_JKL = new AutonConfig("3 Piece JKL", ThreePieceJKL::new,
        "Red Top to J", "Red J to HP", "Red HP to K", "Red K to HP", "Red HP to L");

        AutonConfig BLUE_THREE_HALF_PIECE_JKL = new AutonConfig("3.5 Piece JKL", ThreeHalfPieceJKL::new,
        "Blue Top to J", "Blue J to HP", "Blue HP to K", "Blue K to HP", "Blue HP to L", "Blue L to HP");
        AutonConfig RED_THREE_HALF_PIECE_JKL = new AutonConfig("3.5 Piece JKL", ThreeHalfPieceJKL::new,
        "Red Top to J", "Red J to HP", "Red HP to K", "Red K to HP", "Red HP to L", "Blue L to HP");

        AutonConfig BLUE_FOUR_PIECE_JKLA = new AutonConfig("4 Piece JKLA", FourPieceJKLA::new,
        "Blue Top to J", "Blue J to HP", "Blue HP to K", "Blue K to HP", "Blue HP to L", "Blue L to HP","Red HP to A");
        AutonConfig RED_FOUR_PIECE_JKLA = new AutonConfig("4 Piece JKLA", FourPieceJKLA::new,
        "Red Top to J", "Red J to HP", "Red HP to K", "Red K to HP", "Red HP to L", "Blue L to HP", "Red HP to A");
        
        BLUE_ONE_PIECE_H.registerBlue(autonChooser);
        RED_ONE_PIECE_H.registerRed(autonChooser);
       
        BLUE_ONE_PIECE_J.registerBlue(autonChooser);
        RED_ONE_PIECE_J.registerRed(autonChooser);

        BLUE_TWO_PIECE_JK.registerBlue(autonChooser);
        RED_TWO_PIECE_JK.registerRed(autonChooser);

        BLUE_THREE_PIECE_JKL.registerBlue(autonChooser);
        RED_THREE_PIECE_JKL.registerRed(autonChooser);

        BLUE_THREE_HALF_PIECE_JKL.registerBlue(autonChooser);
        RED_THREE_HALF_PIECE_JKL.registerRed(autonChooser);

        BLUE_FOUR_PIECE_JKLA.registerBlue(autonChooser);
        RED_FOUR_PIECE_JKLA.registerRed(autonChooser);

        /** BOTTOM AUTONS **/

        AutonConfig BLUE_ONE_PIECE_G = new AutonConfig("1 Piece G", OnePieceG::new,
        "Blue Mid Bottom to G");
        AutonConfig RED_ONE_PIECE_G = new AutonConfig("1 Piece G", OnePieceG::new,
        "Red Mid Bottom to G");
        AutonConfig BLUE_ONE_PIECE_E = new AutonConfig("1 Piece E", OnePieceE::new,
        "Blue Bottom to E");
        AutonConfig RED_ONE_PIECE_E = new AutonConfig("1 Piece E", OnePieceE::new,
        "Red Bottom to E");

        AutonConfig BLUE_TWO_PIECE_ED = new AutonConfig("2 Piece ED", TwoPieceED::new,
        "Blue Bottom to E", "Blue E to HP", "Blue HP to D");
        AutonConfig RED_TWO_PIECE_ED = new AutonConfig("2 Piece ED", TwoPieceED::new,
        "Red Bottom to E", "Red E to HP", "Red HP to D");

        AutonConfig BLUE_THREE_PIECE_EDC = new AutonConfig("3 Piece EDC", ThreePieceEDC::new,
        "Blue Bottom to E", "Blue E to HP", "Blue HP to D", "Blue D to HP", "Blue HP to C");
        AutonConfig RED_THREE_PIECE_EDC = new AutonConfig("3 Piece EDC", ThreePieceEDC::new,
        "Red Bottom to E", "Red E to HP", "Red HP to D", "Red D to HP", "Red HP to C");

        AutonConfig BLUE_THREE_HALF_PIECE_EDC = new AutonConfig("3.5 Piece EDC", ThreeHalfPieceEDC::new,
        "Blue Bottom to E", "Blue E to HP", "Blue HP to D", "Blue D to HP", "Blue HP to C", "Blue C to HP");
        AutonConfig RED_THREE_HALF_PIECE_EDC = new AutonConfig("3.5 Piece EDC", ThreeHalfPieceEDC::new,
        "Red Bottom to E", "Red E to HP", "Red HP to D", "Red D to HP", "Red HP to C", "Blue C to HP");

        AutonConfig BLUE_FOUR_PIECE_EDCB = new AutonConfig("4 Piece EDCB", FourPieceEDCB::new,
        "Blue Bottom to E", "Blue E to HP", "Blue HP to D", "Blue D to HP", "Blue HP to C", "Blue C to HP","Red HP to B");
        AutonConfig RED_FOUR_PIECE_EDCB = new AutonConfig("4 Piece EDCB", FourPieceEDCB::new,
        "Red Bottom to E", "Red E to HP", "Red HP to D", "Red D to HP", "Red HP to C", "Blue C to HP", "Red HP to B");

        BLUE_ONE_PIECE_G.registerBlue(autonChooser);
        RED_ONE_PIECE_G.registerRed(autonChooser);
       
        BLUE_ONE_PIECE_E.registerBlue(autonChooser);
        RED_ONE_PIECE_E.registerRed(autonChooser);

        BLUE_TWO_PIECE_ED.registerBlue(autonChooser);
        RED_TWO_PIECE_ED.registerRed(autonChooser);

        BLUE_THREE_PIECE_EDC.registerBlue(autonChooser);
        RED_THREE_PIECE_EDC.registerRed(autonChooser);

        BLUE_THREE_HALF_PIECE_EDC.registerBlue(autonChooser);
        RED_THREE_HALF_PIECE_EDC.registerRed(autonChooser);

        BLUE_FOUR_PIECE_EDCB.registerBlue(autonChooser);
        RED_FOUR_PIECE_EDCB.registerRed(autonChooser);

        /** TESTS **/

        AutonConfig BLUE_MOBILITY = new AutonConfig("Mobility", Mobility::new,
        "Mobility");
        AutonConfig RED_MOBILITY = new AutonConfig("Mobility", Mobility::new, 
        "Mobility");
        AutonConfig STRAIGHT_LINE_TEST = new AutonConfig("Straight Line Test", StraightLineTest::new,
        "Straight Line");
        AutonConfig CURVY_LINE_TEST = new AutonConfig("Curvy Line Test", CurvyLineTest::new,
        "Curvy Line");
        AutonConfig SQUARE_TEST = new AutonConfig("Square Test", SquareTest::new,
        "Square Top", "Square Right", "Square Bottom", "Square Left");
        AutonConfig RSQUARE_TEST = new AutonConfig("RSquare Test", RSquareTest::new,
        "RSquare Top", "RSquare Right", "RSquare Bottom", "RSquare Left");

        BLUE_MOBILITY.registerBlue(autonChooser);
        RED_MOBILITY.registerRed(autonChooser);
        STRAIGHT_LINE_TEST.registerRed(autonChooser);
        CURVY_LINE_TEST.registerRed(autonChooser);
        SQUARE_TEST.registerRed(autonChooser);
        RSQUARE_TEST.registerRed(autonChooser);

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
