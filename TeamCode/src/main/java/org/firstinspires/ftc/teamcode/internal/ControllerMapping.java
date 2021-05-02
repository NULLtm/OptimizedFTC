package org.firstinspires.ftc.teamcode.internal;


import java.util.HashMap;

/**
 * Might be implemented in the future--stay tuned
 */
public interface ControllerMapping {

    /**
     * Build custom controls by inputting the name and control used for each hardware component
     * @return The mapping of device names to Keys on controller
     */
    HashMap<String, OptimizedController.Key> initializeMapping(HashMap<String, OptimizedController.Key> mapping);
}
