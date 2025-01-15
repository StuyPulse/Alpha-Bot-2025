/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import java.lang.annotation.ElementType;

import com.stuypulse.robot.commands.Elevator.ElevatorToBottom;
import com.stuypulse.robot.commands.Elevator.ElevatorToLvl1;
import com.stuypulse.robot.commands.Elevator.ElevatorToLvl2;
import com.stuypulse.robot.commands.Elevator.ElevatorToLvl3;
import com.stuypulse.robot.commands.Elevator.ElevatorToLvl4;
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.Elevator.Elevator;
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
    public final Elevator elevator = Elevator.getInstance();
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

    private void configureDriverBindings() {
        
    }

    private void configureOperatorBindings() {

        operator.getDPadDown()
            .onTrue(new ElevatorToLvl1());

        operator.getDPadLeft()
            .onTrue(new ElevatorToLvl2());
            
        operator.getDPadRight()
            .onTrue(new ElevatorToLvl3());    

        operator.getDPadUp()
            .onTrue(new ElevatorToLvl4());

        operator.getLeftButton()
            .onTrue(new ElevatorToBottom());
        
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
