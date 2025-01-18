/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import javax.management.openmbean.OpenType;

// Algae Commands
import com.stuypulse.robot.commands.algae.AlgaeGroundPickup;
import com.stuypulse.robot.commands.algae.AlgaeL2;
import com.stuypulse.robot.commands.algae.AlgaeL3;
import com.stuypulse.robot.commands.algae.AlgaeProcessorScore;
import com.stuypulse.robot.commands.algae.AlgaeReefKnockoff;
import com.stuypulse.robot.commands.algae.AlgaeStopRoller;
import com.stuypulse.robot.commands.algae.AlgaeStow;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.algae.Algae;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);
    
    // Subsystem
    public final Algae algae = Algae.getInstance();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {}

    /***************/
    /*** BUTTONS ***/
    /***************/


    private void configureButtonBindings() {
        configureDriverBindings();
        configureOperatorBindings();
    }

    private void configureDriverBindings() {}

    private void configureOperatorBindings() {

        operator.getBottomButton()
            .onTrue(new AlgaeStow());
        
        operator.getTopButton()
            .onTrue(new AlgaeGroundPickup());
        
        operator.getLeftButton()
            .onTrue(new AlgaeL2());
        
        operator.getRightButton()
            .onTrue(new AlgaeL3());

        operator.getLeftBumper()
            .onTrue(new AlgaeProcessorScore());

        operator.getRightBumper()
            .onTrue(new AlgaeReefKnockoff());

        operator.getLeftMenuButton()
            .onTrue(new AlgaeStopRoller());
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
