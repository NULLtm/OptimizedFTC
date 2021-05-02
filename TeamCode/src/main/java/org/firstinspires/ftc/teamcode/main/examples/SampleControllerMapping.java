package org.firstinspires.ftc.teamcode.main.examples;

import org.firstinspires.ftc.teamcode.internal.ControllerMapping;
import org.firstinspires.ftc.teamcode.internal.Experimental;
import org.firstinspires.ftc.teamcode.internal.OptimizedController;

import java.util.HashMap;

@Experimental
public class SampleControllerMapping implements ControllerMapping {
    @Override
    public HashMap<String, OptimizedController.Key> initializeMapping(HashMap<String, OptimizedController.Key> mapping) {

        // Adding controls
        mapping.put("intakeIn", OptimizedController.Key.LEFT_TRIGGER);
        mapping.put("intakeOut", OptimizedController.Key.RIGHT_TRIGGER);
        mapping.put("outtakeManual", OptimizedController.Key.LEFT_TRIGGER);
        mapping.put("hopperManual", OptimizedController.Key.B);
        mapping.put("outtakeAutomatic", OptimizedController.Key.A);
        mapping.put("wobbleUp", OptimizedController.Key.DPAD_UP);
        mapping.put("wobbleDown", OptimizedController.Key.DPAD_DOWN);
        mapping.put("wobbleOpen", OptimizedController.Key.DPAD_LEFT);
        mapping.put("wobbleClose", OptimizedController.Key.DPAD_RIGHT);
        mapping.put("powershotAutomatic", OptimizedController.Key.Y);
        mapping.put("wobbleMotor", OptimizedController.Key.LEFT_STICK_Y);

        return mapping;
    }
}
