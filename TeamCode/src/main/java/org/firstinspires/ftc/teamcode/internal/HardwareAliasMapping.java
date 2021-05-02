package org.firstinspires.ftc.teamcode.internal;


import java.util.HashMap;
import java.util.List;

/**
 * Might be implemented in the future--stay tuned
 */
public interface HardwareAliasMapping {

    /**
     * Build custom controls by inputting the name and control used for each hardware component
     * @return The mapping of device names to Keys on controller
     */
    HashMap<String, List<String>> initializeMapping(HashMap<String, List<String>> mapping);
}
