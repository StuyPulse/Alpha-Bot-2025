 /************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.auton.FourPiece;
import com.stuypulse.robot.commands.auton.Mobility;
import com.stuypulse.robot.commands.auton.OnePiece;
import com.stuypulse.robot.commands.auton.ThreeHalfPiece;
import com.stuypulse.robot.commands.auton.ThreePiece;
import com.stuypulse.robot.commands.auton.TwoPiece;
import com.stuypulse.robot.commands.auton.sequences.AcquireSequence;
import com.stuypulse.robot.commands.Elevator.ElevatorToBottom;
import com.stuypulse.robot.commands.Elevator.ElevatorToLvl1;
import com.stuypulse.robot.commands.Elevator.ElevatorToLvl2;
import com.stuypulse.robot.commands.Elevator.ElevatorToLvl3;
import com.stuypulse.robot.commands.Elevator.ElevatorToLvl4;
import com.stuypulse.robot.commands.funnel.FunnelDeacquire;
import com.stuypulse.robot.commands.funnel.FunnelStop;
import com.stuypulse.robot.commands.shooter.ShooterShoot;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.Elevator.Elevator;
import com.stuypulse.robot.subsystems.funnel.Funnel;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.AprilTagVision;
import com.stuypulse.robot.util.PathUtil.AutonConfig;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

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
    public final AprilTagVision vision = AprilTagVision.getInstance();
    public final Odometry odometry = Odometry.getInstance();
    public final Shooter shooter = Shooter.getInstance();
    public final Funnel funnel = Funnel.getInstance();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot Container

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        //configureAutons();
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
        
        driver.getTopButton()
            .whileTrue(new ElevatorToLvl4());
        
        driver.getRightButton()
            .whileTrue(new ElevatorToLvl3());

        driver.getLeftButton()
            .whileTrue(new ElevatorToLvl2());

        driver.getBottomButton()
            .whileTrue(new ElevatorToLvl1());

        driver.getDPadDown()
            .whileTrue(new ElevatorToBottom());

        driver.getDPadUp()
            .whileTrue(new FunnelDeacquire());

        driver.getLeftBumper()
            .whileTrue(new AcquireSequence())
            .whileFalse(new FunnelStop());

        driver.getRightBumper()
            .whileTrue(new ShooterShoot());

    }

    /**************/
    /*** AUTONS ***/
    /**************/

    // public void configureAutons() {
    //     swerve.configureAutoBuilder();

    //     autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());

    //     /** TOP AUTONS **/

    //     AutonConfig BLUE_ONE_PIECE_H = new AutonConfig("1 Piece H", OnePiece::new,
    //     "Blue Mid Top to H");
    //     AutonConfig RED_ONE_PIECE_H = new AutonConfig("1 Piece H", OnePiece::new,
    //     "Red Mid Top to H");

    //     AutonConfig BLUE_ONE_PIECE_J = new AutonConfig("1 Piece J", OnePiece::new,
    //     "Blue Top to J");
    //     AutonConfig RED_ONE_PIECE_J = new AutonConfig("1 Piece J", OnePiece::new,
    //     "Red Top to J");

    //     AutonConfig BLUE_TWO_PIECE_JK = new AutonConfig("2 Piece JK", TwoPiece::new,
    //     "Blue Top to J", "Blue J to HP", "Blue HP to K");
    //     AutonConfig RED_TWO_PIECE_JK = new AutonConfig("2 Piece JK", TwoPiece::new,
    //     "Red Top to J", "Red J to HP", "Red HP to K");

    //     AutonConfig BLUE_THREE_PIECE_JKL = new AutonConfig("3 Piece JKL", ThreePiece::new,
    //     "Blue Top to J", "Blue J to HP", "Blue HP to K", "Blue K to HP", "Blue HP to L");
    //     AutonConfig RED_THREE_PIECE_JKL = new AutonConfig("3 Piece JKL", ThreePiece::new,
    //     "Red Top to J", "Red J to HP", "Red HP to K", "Red K to HP", "Red HP to L");

    //     AutonConfig BLUE_THREE_HALF_PIECE_JKL = new AutonConfig("3.5 Piece JKL", ThreeHalfPiece::new,
    //     "Blue Top to J", "Blue J to HP", "Blue HP to K", "Blue K to HP", "Blue HP to L", "Blue L to HP");
    //     AutonConfig RED_THREE_HALF_PIECE_JKL = new AutonConfig("3.5 Piece JKL", ThreeHalfPiece::new,
    //     "Red Top to J", "Red J to HP", "Red HP to K", "Red K to HP", "Red HP to L", "Blue L to HP");

    //     AutonConfig BLUE_FOUR_PIECE_JKLA = new AutonConfig("4 Piece JKLA", FourPiece::new,
    //     "Blue Top to J", "Blue J to HP", "Blue HP to K", "Blue K to HP", "Blue HP to L", "Blue L to HP","Red HP to A");
    //     AutonConfig RED_FOUR_PIECE_JKLA = new AutonConfig("4 Piece JKLA", FourPiece::new,
    //     "Red Top to J", "Red J to HP", "Red HP to K", "Red K to HP", "Red HP to L", "Blue L to HP", "Red HP to A");
        
    //     BLUE_ONE_PIECE_H.registerBlue(autonChooser);
    //     RED_ONE_PIECE_H.registerRed(autonChooser);
       
    //     BLUE_ONE_PIECE_J.registerBlue(autonChooser);
    //     RED_ONE_PIECE_J.registerRed(autonChooser);

    //     BLUE_TWO_PIECE_JK.registerBlue(autonChooser);
    //     RED_TWO_PIECE_JK.registerRed(autonChooser);

    //     BLUE_THREE_PIECE_JKL.registerBlue(autonChooser);
    //     RED_THREE_PIECE_JKL.registerRed(autonChooser);

    //     BLUE_THREE_HALF_PIECE_JKL.registerBlue(autonChooser);
    //     RED_THREE_HALF_PIECE_JKL.registerRed(autonChooser);

    //     BLUE_FOUR_PIECE_JKLA.registerBlue(autonChooser);
    //     RED_FOUR_PIECE_JKLA.registerRed(autonChooser);

    //     /** BOTTOM AUTONS **/

    //     AutonConfig BLUE_ONE_PIECE_G = new AutonConfig("1 Piece G", OnePiece::new,
    //     "Blue Mid Bottom to G");
    //     AutonConfig RED_ONE_PIECE_G = new AutonConfig("1 Piece G", OnePiece::new,
    //     "Red Mid Bottom to G");
    //     AutonConfig BLUE_ONE_PIECE_E = new AutonConfig("1 Piece E", OnePiece::new,
    //     "Blue Bottom to E");
    //     AutonConfig RED_ONE_PIECE_E = new AutonConfig("1 Piece E", OnePiece::new,
    //     "Red Bottom to E");

    //     AutonConfig BLUE_TWO_PIECE_ED = new AutonConfig("2 Piece ED", TwoPiece::new,
    //     "Blue Bottom to E", "Blue E to HP", "Blue HP to D");
    //     AutonConfig RED_TWO_PIECE_ED = new AutonConfig("2 Piece ED", TwoPiece::new,
    //     "Red Bottom to E", "Red E to HP", "Red HP to D");

    //     AutonConfig BLUE_THREE_PIECE_EDC = new AutonConfig("3 Piece EDC", ThreePiece::new,
    //     "Blue Bottom to E", "Blue E to HP", "Blue HP to D", "Blue D to HP", "Blue HP to C");
    //     AutonConfig RED_THREE_PIECE_EDC = new AutonConfig("3 Piece EDC", ThreePiece::new,
    //     "Red Bottom to E", "Red E to HP", "Red HP to D", "Red D to HP", "Red HP to C");

    //     AutonConfig BLUE_THREE_HALF_PIECE_EDC = new AutonConfig("3.5 Piece EDC", ThreeHalfPiece::new,
    //     "Blue Bottom to E", "Blue E to HP", "Blue HP to D", "Blue D to HP", "Blue HP to C", "Blue C to HP");
    //     AutonConfig RED_THREE_HALF_PIECE_EDC = new AutonConfig("3.5 Piece EDC", ThreeHalfPiece::new,
    //     "Red Bottom to E", "Red E to HP", "Red HP to D", "Red D to HP", "Red HP to C", "Blue C to HP");

    //     AutonConfig BLUE_FOUR_PIECE_EDCB = new AutonConfig("4 Piece EDCB", FourPiece::new,
    //     "Blue Bottom to E", "Blue E to HP", "Blue HP to D", "Blue D to HP", "Blue HP to C", "Blue C to HP","Red HP to B");
    //     AutonConfig RED_FOUR_PIECE_EDCB = new AutonConfig("4 Piece EDCB", FourPiece::new,
    //     "Red Bottom to E", "Red E to HP", "Red HP to D", "Red D to HP", "Red HP to C", "Blue C to HP", "Red HP to B");

    //     BLUE_ONE_PIECE_G.registerBlue(autonChooser);
    //     RED_ONE_PIECE_G.registerRed(autonChooser);
       
    //     BLUE_ONE_PIECE_E.registerBlue(autonChooser);
    //     RED_ONE_PIECE_E.registerRed(autonChooser);

    //     BLUE_TWO_PIECE_ED.registerBlue(autonChooser);
    //     RED_TWO_PIECE_ED.registerRed(autonChooser);

    //     BLUE_THREE_PIECE_EDC.registerBlue(autonChooser);
    //     RED_THREE_PIECE_EDC.registerRed(autonChooser);

    //     BLUE_THREE_HALF_PIECE_EDC.registerBlue(autonChooser);
    //     RED_THREE_HALF_PIECE_EDC.registerRed(autonChooser);

    //     BLUE_FOUR_PIECE_EDCB.registerBlue(autonChooser);
    //     RED_FOUR_PIECE_EDCB.registerRed(autonChooser);

    //     /** TESTS **/

    //     AutonConfig BLUE_MOBILITY = new AutonConfig("Mobility", Mobility::new,
    //     "Blue Mobility");
    //     AutonConfig RED_MOBILITY = new AutonConfig("Mobility", Mobility::new, 
    //     "Red Mobility");

    //     BLUE_MOBILITY.registerBlue(autonChooser);
    //     RED_MOBILITY.registerRed(autonChooser);

    //     SmartDashboard.putData("Autonomous", autonChooser);
    // }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
