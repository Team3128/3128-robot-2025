package frc.team3128.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import common.hardware.input.NAR_XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.RobotContainer;



public class CmdManager {

    private static NAR_XboxController controller = RobotContainer.controller;

    public static Command vibrateController(){
        return startEnd(()-> controller.startVibrate(), ()-> controller.stopVibrate()).withTimeout(1);
    }
}