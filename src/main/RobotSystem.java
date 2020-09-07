package main;

import java.awt.EventQueue;
import simulator.Controller;

/**
 * This Java file represent the main.java file and will detect if it's a real run or simulated run.
 */
public class RobotSystem {
	
	private static boolean _isRealRun = false;

	public static void main(String[] args) {
		EventQueue.invokeLater(new Runnable() {
			public void run() {
				new RobotSystem();
			}
		});
	}
	
	public RobotSystem() {
		Controller c = Controller.getInstance();
		c.run();
	}
	
	public static boolean isRealRun() {
		return _isRealRun;
	}

}
