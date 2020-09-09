package algorithms;

import java.util.ArrayList;
import java.util.Collections;
import datatypes.Movement;
import datatypes.Orientation;
import simulator.arena.Arena;
import simulator.robot.Robot;

public class FloodFillingPathFinder {
	
	private static FloodFillingPathFinder _instance;
	private VirtualMap _virtualMap;
	private Robot _robot;
	
	public static FloodFillingPathFinder getInstance() {
        if (_instance == null) {
            _instance = new FloodFillingPathFinder();
        }
        return _instance;
    }

}
