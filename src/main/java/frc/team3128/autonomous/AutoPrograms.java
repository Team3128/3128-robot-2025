package frc.team3128.autonomous;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import common.utility.Log;
import common.utility.narwhaldashboard.NarwhalDashboard;


/**
 * Class to store information about autonomous routines.
 * @author Daniel Wang
 */

public class AutoPrograms {

    private HashMap<String, Command> autoMap = new HashMap<String, Command>();
    private HashMap<String, Command> pathMap = new HashMap<String, Command>();
    private static AutoPrograms instance;

    public AutoPrograms() {
        initAutoSelector();
    }

    public static synchronized AutoPrograms getInstance() {
        if (instance == null) instance = new AutoPrograms();
        return instance;
    }

    private void initAutoSelector() {
        final String[] autoStrings = new String[] {"default"};
        final String[] pathStrings = new String[] {};
        
        // NarwhalDashboard.getInstance().addAutos(autoStrings);
        for (String auto : autoStrings) {
            if (auto.equals("default")) continue;
            autoMap.put(auto, none());
        }
        for (String path : pathStrings) {
            pathMap.put(path, none());
        }
    }

    public Command getAuto(String name) {
        return autoMap.get(name);
    }

    public Command getPath(String name) {
        return pathMap.get(name);
    }

    public Command getAutonomousCommand() {
        String selectedAutoName = ""; //NarwhalDashboard.getInstance().getSelectedAuto();
        String hardcode = "";
        
        Command autoCommand;
        if (selectedAutoName == null) {
            selectedAutoName = hardcode;
        }
        else if (selectedAutoName.equals("default")) {
            defaultAuto();
        }
        autoCommand = autoMap.get(selectedAutoName);

        Log.info("AUTO_SELECTED", selectedAutoName);
        return autoCommand.beforeStarting(reset());
    }

    private Command defaultAuto(){
        return none();
    }

    private Command reset() {
        return none();
    }
}