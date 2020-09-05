package simulator.robot;

import java.io.IOException;
import datatypes.Message;
import datatypes.Orientation;
import main.RobotSystem;
import simulator.Controller;
import simulator.arena.Arena;
import tcpcomm.PCClient;

/* This Java file handles the simulator robot settings */

public class Robot {
	
	private static Robot _instance;
	private int _speed;
	private int _stepsSinceLastCalibration;
	
	//Constructor that will handle the calibration counter.
	private Robot() {
		_stepsSinceLastCalibration = 0;
	}
	
	//This function will generate a Robot instance.
	public static Robot getInstance() {
		if (_instance == null) {
			_instance = new Robot();
		}
		return _instance;
	}
	
	//This function will change the speed of the simulator robot.
	public void setSpeed(int speed) {
		_speed = speed;
	}
	
	//This function will return you the number of steps since last calibration.
	public int getStepsSinceLastCalibration(){
		return _stepsSinceLastCalibration;
	}
	
	//This function will reset the calibration steps to 0.
	public void resetStepsSinceLastCalibration(){
		_stepsSinceLastCalibration = 0;
	}
	
	//This function handles the front sensor.
	public int senseFront(int[] sensorPosition, Orientation robotOrientation) {
		Arena arena = Arena.getInstance();
		int numOfClearGrids;
		Orientation sensorOri = robotOrientation;
		numOfClearGrids = arena.getNumOfClearGrids(sensorPosition, sensorOri);
		if (numOfClearGrids > Sensor.SHORT_RANGE) {
			numOfClearGrids = Sensor.SHORT_RANGE;
		}
		return numOfClearGrids;
	}
	
	//This function handles the front side sensor.
	public int senseSideFront (int[] sensorPosition, Orientation robotOrientation) {
		Arena arena = Arena.getInstance();
		int numOfClearGrids;
		Orientation sensorOri = robotOrientation;
		numOfClearGrids = arena.getNumOfClearGrids(sensorPosition, sensorOri);
		if (numOfClearGrids > Sensor.SHORT_RANGE) {
			numOfClearGrids = Sensor.SHORT_RANGE;
		}
		return numOfClearGrids;
	}
	
	//This function handles the left sensor.
	public int senseLeft(int[] sensorPosition, Orientation robotOrientation) {
		Arena arena = Arena.getInstance();
		int numOfClearGrids;
		Orientation sensorOri;
		switch (robotOrientation) {
			case NORTH:
				sensorOri = Orientation.WEST;
				break;
			case SOUTH:
				sensorOri = Orientation.EAST;
				break;
			case EAST:
				sensorOri = Orientation.NORTH;
				break;
			case WEST:
				sensorOri = Orientation.SOUTH;
				break;
			default:
				sensorOri = null;
		}
		numOfClearGrids = arena.getNumOfClearGrids(sensorPosition, sensorOri);
		if (numOfClearGrids > Sensor.LONG_RANGE) {
			numOfClearGrids = Sensor.LONG_RANGE;
		}
		return numOfClearGrids;
	}
	
	//This function handles the right sensor.
	public int senseRight(int[] sensorPosition, Orientation robotOrientation) {
		Arena arena = Arena.getInstance();
		int numOfClearGrids;
		Orientation sensorOri;
		switch (robotOrientation) {
			case NORTH:
				sensorOri = Orientation.EAST;
				break;
			case SOUTH:
				sensorOri = Orientation.WEST;
				break;
			case EAST:
				sensorOri = Orientation.SOUTH;
				break;
			case WEST:
				sensorOri = Orientation.NORTH;
				break;
			default:
				sensorOri = null;
		}
		numOfClearGrids = arena.getNumOfClearGrids(sensorPosition, sensorOri);
		if (numOfClearGrids > Sensor.SHORT_RANGE) {
			numOfClearGrids = Sensor.SHORT_RANGE;
		}
		return numOfClearGrids;
	}
	
	//This function handles rotate right of the simulator robot.
	public void turnRight() {
		Controller controller = Controller.getInstance();
		PCClient pcClient = controller.getPCClient();
		if (!RobotSystem.isRealRun()) {
			int stepTime = 1000 / _speed;
			try {
				Thread.sleep(stepTime);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		} else {
			try {
				pcClient.sendMessage(Message.TURN_RIGHT + Message.SEPARATOR);
	
				String feedback = pcClient.readMessage();
				while (!feedback.equals(Message.DONE)) {
					feedback = pcClient.readMessage();
				}
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
		controller.turnRobotRight();
	}
	
	//This function handles rotate left of the simulator robot.
	public void turnLeft() {
		Controller controller = Controller.getInstance();
		PCClient pcClient = controller.getPCClient();
		if (!RobotSystem.isRealRun()) {
			int stepTime = 1000 / _speed;
			try {
				Thread.sleep(stepTime);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		} else {
			try {
				pcClient.sendMessage(Message.TURN_LEFT + Message.SEPARATOR);
				String feedback = pcClient.readMessage();
				while (!feedback.equals(Message.DONE)) {
					feedback = pcClient.readMessage();
				}
			} catch (IOException e) {
				e.printStackTrace();
			}
		}

		controller.turnRobotLeft();
	}

	//This function handles movement forward.
	public void moveForward() {
		Controller controller = Controller.getInstance();
		PCClient pcClient = controller.getPCClient();
		if (!RobotSystem.isRealRun()) {
			int stepTime = 1000 / _speed;
			try {
				Thread.sleep(stepTime);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		} else {
			try {
				pcClient.sendMessage(Message.MOVE_FORWARD + Message.SEPARATOR);
				String feedback = pcClient.readMessage();
				while (!feedback.equals(Message.DONE)) {
					feedback = pcClient.readMessage();
				}
			} catch (IOException e) {
				e.printStackTrace();
			}
			_stepsSinceLastCalibration ++;
		}
		controller.moveRobotForward();
	}

	//This function overloads the previous function by allowing you to select the amount of grids to move forward.
	public void moveForward(int count) {
		Controller controller = Controller.getInstance();
		PCClient pcClient = controller.getPCClient();
		if (!RobotSystem.isRealRun()) {
			int stepTime = 1000 / _speed;
			for (int i = 0; i < count; i++) {
				try {
					Thread.sleep(stepTime);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
		
				controller.moveRobotForward();
			}
		} else {
			try {
				if (count >= 10) {
					pcClient.sendMessage(Message.MOVE_FORWARD + count % 10 + count / 10 +  "|");
				} else {
					pcClient.sendMessage(Message.MOVE_FORWARD + count + Message.SEPARATOR);
				}

				String feedback = pcClient.readMessage();
				for (int i = 0; i < count; i++) {
					while (!feedback.equals(Message.DONE)) {
						feedback = pcClient.readMessage();
					}
					controller.moveRobotForward();
				}
				
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}

	//This function handles the robot position calibration.
	public void calibrateRobotPosition() {
		Controller controller = Controller.getInstance();
		PCClient pcClient = controller.getPCClient();
		try {
			pcClient.sendMessage(Message.CALIBRATE + Message.SEPARATOR);
			String feedback = pcClient.readMessage();
			while (!feedback.equals(Message.DONE)) {
				feedback = pcClient.readMessage();
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	//This function handles the robot calibration at the start zone.
	public Orientation calibrateAtStartZone(Orientation ori) {	
		switch (ori) {
			case NORTH:
				turnLeft();
				calibrateRobotPosition();
				turnLeft();
				calibrateRobotPosition();
				turnRight();
				turnRight();
				break;
			case SOUTH:
				calibrateRobotPosition();
				turnRight();
				calibrateRobotPosition();
				turnRight();			
				break;
			case EAST:
				turnRight();
				calibrateRobotPosition();
				turnRight();
				calibrateRobotPosition();
				turnRight();
				break;
			case WEST:
				calibrateRobotPosition();
				turnLeft();
				calibrateRobotPosition();
				turnRight();
				turnRight();
		}
		return Orientation.NORTH;
	}
}
