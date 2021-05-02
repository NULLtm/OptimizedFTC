package org.firstinspires.ftc.teamcode.main.examples;

import org.firstinspires.ftc.teamcode.internal.ControllerMapping;
import org.firstinspires.ftc.teamcode.internal.Experimental;
import org.firstinspires.ftc.teamcode.internal.HardwareAliasMapping;
import org.firstinspires.ftc.teamcode.internal.OptimizedController;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

@Experimental
public class SampleHardwareAliasMapping implements HardwareAliasMapping {

    @Override
    public HashMap<String, List<String>> initializeMapping(HashMap<String, List<String>> mapping) {


        // For each hardware component actual map name -- add aliases you might want to use
        mapping.put("xEncoder", Arrays.asList("outtakeMotor1", "outtake1"));
        mapping.put("yEncoder", Arrays.asList("wobbleMotor"));

        return mapping;
    }
}
