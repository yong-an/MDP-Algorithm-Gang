package algorithms;

import java.io.IOException;
import java.time.LocalTime;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Date;
import java.util.List;

import java.util.HashMap;

import datatypes.ImageRef;
import datatypes.Message;
import datatypes.Movement;
import datatypes.Orientation;
import datatypes.SensorPosition;
import main.RobotSystem;
import simulator.Controller;
import simulator.arena.Arena;
import simulator.robot.Robot;
import simulator.robot.Sensor;
import tcpcomm.PCClient;
import tcpcomm.ThreadPoolImage;

public class MazeExplorer {

	public static final int RIGHT_CHECK = -2;
	public static final int UNEXPLORED = -1;
	public static final int IS_OBSTACLE = 1;
	public static final int IS_EMPTY = 0;

	private static final int RIGHT_NO_ACCESS = -1;
	private static final int RIGHT_UNSURE_ACCESS = -2;
	private static final int RIGHT_CAN_ACCESS = -3;

	public static final int[] GOAL = { 13, 18 };
	public static final int[] START = { 1, 1 };

	private static final int INVALID_SENSOR_VALUE = -1;
	private static final int CALIBRATION_THRESHOLD = 3;
	private static final int RIGHT_CALIBRATION_THRESHOLD = 2;

	private static MazeExplorer _instance;
	private Boolean[][] _isExplored;
	private int[][] _mazeRef, rightWallRef, imageRef, weightageRef;
	private Robot _robot;
	private int[] _robotPosition;
	private Orientation _robotOrientation;
	private boolean _hasExploredTillGoal;
	private boolean startImageRun = true, nextRun = false, isFastestPathBack = false, leftObstacleSelected = false;
	private Path _fastestPathBack = null;
	private AStarPathFinder _pathFinder;
	private int leftTurnCounter = 0;
	private int leftCountdown = 2;
	private int[] lastCalibrate = new int[2];
	private boolean leftGotObstacle = false;
	private ArrayList<ImageRef> arrayListOfImageRefs = new ArrayList<ImageRef>();
	private ArrayList<ImageRef> arrayListOfImageRefsExploration = new ArrayList<ImageRef>();
	private boolean leftWallKiss = false;
	
	private MazeExplorer() {

	}

	public void setLeftCoundown() {
		leftCountdown--;
	}

	public void setLeftCountdownBack() {
		leftCountdown = 2;
	}

	public boolean getleftGotObstacle() {
		return leftGotObstacle;
	}

	public void setleftGotObstacle() {
		leftGotObstacle = false;
	}

	public void setOrientation(Orientation orientation) {
		_robotOrientation = orientation;
	}

	public static MazeExplorer getInstance() {
		if (_instance == null) {
			_instance = new MazeExplorer();
		}
		return _instance;
	}

	public int[] getRobotPosition() {
		return _robotPosition;
	}

	private void setRobotPosition(int x, int y) {
		_robotPosition[0] = x;
		_robotPosition[1] = y;
	}

	public Orientation getRobotOrientation() {
		return _robotOrientation;
	}

	public int[][] getMazeRef() {
		return _mazeRef;
	}

	public int[][] getWeightageRef() {
		return weightageRef;
	}

	public void setMazeReObstacle(int x, int y) {
		_mazeRef[x][y] = IS_OBSTACLE;
	}

	public Boolean[][] getIsExplored() {
		return _isExplored;
	}

	public boolean areAllExplored() {
		for (int i = 0; i < Arena.MAP_LENGTH; i++) {
			for (int j = 0; j < Arena.MAP_WIDTH; j++) {
				if (_isExplored[i][j] == false) {
					return false;
				}
			}
		}
		return true;
	}

	public boolean hasExploredTillGoal() {
		return _hasExploredTillGoal;
	}

	public String getP1Descriptor() {
		String P1Binary = "11";
		int value;
		for (int j = 0; j < Arena.MAP_WIDTH; j++) {
			for (int i = 0; i < Arena.MAP_LENGTH; i++) {
				value = (_isExplored[i][j]) ? 1 : 0;
				P1Binary = P1Binary + value;
			}
		}
		P1Binary = P1Binary + "11";
		String temp, P1HexStr = "";
		int index = 0;

		while (index < P1Binary.length()) {
			temp = "";
			for (int i = 0; i < 4; i++) {
				temp = temp + P1Binary.charAt(index);
				index++;
			}
			P1HexStr = P1HexStr + Integer.toString(Integer.parseInt(temp, 2), 16);
		}
		return P1HexStr;
	}

	public String getP2Descriptor() {
		String P2Binary = "";
		for (int j = 0; j < Arena.MAP_WIDTH; j++) {
			for (int i = 0; i < Arena.MAP_LENGTH; i++) {
				if (_mazeRef[i][j] == IS_OBSTACLE) {
					P2Binary = P2Binary + 1;
				} else if (_mazeRef[i][j] == IS_EMPTY) {
					P2Binary = P2Binary + 0;
				}
			}
		}

		int remainder = P2Binary.length() % 8;

		for (int i = 0; i < 8 - remainder; i++) {
			P2Binary = P2Binary + "0";
		}

		String temp, P2HexStr = "";
		int index = 0;

		while (index < P2Binary.length()) {
			temp = "";
			for (int i = 0; i < 4; i++) {
				temp = temp + P2Binary.charAt(index);
				index++;
			}
			P2HexStr = P2HexStr + Integer.toString(Integer.parseInt(temp, 2), 16);
		}

		return P2HexStr;
	}

	public void explore(int[] robotPosition, Orientation robotOrientation) {

		Controller controller = Controller.getInstance();

		// if (!RobotSystem.isRealRun())

		init(robotPosition, robotOrientation);

		for (int i = robotPosition[0] - 1; i <= robotPosition[0] + 1; i++) {
			for (int j = robotPosition[1] - 1; j <= robotPosition[1] + 1; j++) {
				_isExplored[i][j] = true;
				_mazeRef[i][j] = IS_EMPTY;
				rightWallRef[i][j] = IS_EMPTY;
				imageRef[i][j] = IS_EMPTY;
				weightageRef[i][j] = IS_EMPTY;
			}
		}
		setIsExplored(_robotPosition, _robotOrientation, true);
		Thread t1 = new Thread(new ThreadPoolImage());
		t1.start();
		exploreAlongWallClean(GOAL);

		if (!controller.hasReachedTimeThreshold()) {
			_hasExploredTillGoal = true;
			System.out.println("exploreAlongWall Start Start");
			exploreAlongWallClean(START);
			System.out.println("exploreAlongWall Start End");
		} else {
			_hasExploredTillGoal = false; // Timeout before reaching goal
		}

		 hasNextRun();

		if (startImageRun) {
			_robot = Robot.getInstance();
			_robotOrientation = _robot.calibrateAfterExploration(_robotOrientation);
			controller.setRobotOrientation(_robotOrientation);
			_robot.resetStepsSinceLastCalibration();
			findImage();
		}

		// Current robot position is not at start point, find fastest path back to start
		// point
		System.out.println("After end");
		// fastestPathBackToStart();

		/*
		 * if (RobotSystem.isRealRun()) { //Send Arduino the signal that exploration is
		 * done //_robotOrientation = _robot.calibrateAtStartZone(_robotOrientation);
		 * try { PCClient pcClient = PCClient.getInstance();
		 * pcClient.sendMessage(Message.EXPLORE_DONE + Message.SEPARATOR); String
		 * msgExDone = pcClient.readMessage(); while (!msgExDone.equals(Message.DONE)) {
		 * msgExDone = pcClient.readMessage(); } System.out.println("DONEDONEDONE");
		 * pcClient.sendMessageToAndroid(Message.EXPLORE_DONE);
		 * pcClient.sendMsgToRPI(Message.RPI_DONE); System.out.println("DONEDONEDONE2");
		 * 
		 * } catch (IOException e) { e.printStackTrace(); } } else {
		 * adjustOrientationTo(Orientation.NORTH); }
		 */
	}

	private void fastestPathBackToStart() {
		System.out.println("Current pos: " + _robotPosition[0] + "," + _robotPosition[1]);
		if (!isGoalPos(_robotPosition, START)) {
			System.out.println("Heading back to start point " + LocalTime.now());
			AStarPathFinder pathFinder = AStarPathFinder.getInstance();
			// Controller controller = Controller.getInstance();
			// Path backPath = controller.getFastestPathBack();
//			if(backPath != null) {
			Path backPath = pathFinder.findFastestPath(_robotPosition[0], _robotPosition[1], START[0], START[1],
					_mazeRef);
			_robotOrientation = pathFinder.moveRobotAlongFastestPath(backPath, _robotOrientation, false, false, false);
			_robotOrientation = _robot.calibrateAtStartZone(_robotOrientation);
			setRobotPosition(START[0], START[1]);
//			}
		} else
			_robotOrientation = _robot.calibrateAtStartZone(_robotOrientation);
	}

	public void hasNextRun() {
		System.out.println("Start next Run");
		Controller controller = Controller.getInstance();
		boolean end = false;
		HashMap<Integer, int[]> positionHashMap = new HashMap<Integer, int[]>();
		HashMap<Integer, Integer> positionHashMap2 = new HashMap<Integer, Integer>();
		while (!controller.hasReachedTimeThreshold() && !areAllExplored() && !end) {
			nextRun = true;
			end = ExploreNextRound(_robotPosition, positionHashMap, positionHashMap2);
		}
	}

	private int[] getPos() {
		int[] obsPos = new int[2];

		for (int obsX = 0; obsX < Arena.MAP_LENGTH; obsX++) {
			for (int obsY = 0; obsY < Arena.MAP_WIDTH; obsY++) {
				if (imageRef[obsX][obsY] == IS_OBSTACLE) {
					obsPos[0] = obsX;
					obsPos[1] = obsY;
					return obsPos;
				}
			}
		}
		return null;
	}

	public void sendObsPosLeft(int[] robotPosition, Orientation ori, int x, int y, int blockAway) {

		Robot robot = Robot.getInstance();
		String msg = "";

		if (leftCountdown == 0 && leftGotObstacle == true) {
			robot.turnRight();
			robot.turnRight();

			msg = x + "_" + y + "_" + ori;
			System.out.println("Send to RPI Left Wall Image: " + msg);
			if (RobotSystem.isRealRun()) {
				Controller controller = Controller.getInstance();
				PCClient pcClient = controller.getPCClient();

				try {
					Date startDate = new Date();
					Date endDate = new Date();
					int numSeconds = 0;
					pcClient.sendMsgToRPI(msg);
					System.out.println("Sent Take Picture Command");
					String feedback = pcClient.readMessage();
					while (!feedback.equals(Message.DONE)) {
						feedback = pcClient.readMessage();
						System.out.println("Reading Picture Taking Command");
						endDate = new Date();
						numSeconds = (int) ((endDate.getTime() - startDate.getTime()) / 1000);
						System.out.println("Picture Taking Command Timing: " + numSeconds);
						if (((numSeconds % 2) == 0) || numSeconds < 10)
							pcClient.sendMsgToRPI(msg);
						else if (numSeconds >= 10)
							break;

					}
					System.out.println("Outside While Loop");
				} catch (IOException e) {
					e.printStackTrace();
				}
				System.out.println("Picture Command Completed");
			}
			robot.turnRight();
			robot.turnRight();
			leftCountdown = 3;
			leftGotObstacle = false;
		}

	}

	public void sendObstaclePos(int[] robotPosition, Orientation ori) throws IOException {

		String msg = "";

		if (ori == Orientation.NORTH && (robotPosition[0] + 2) != 15)
			msg = (robotPosition[0] + 2) + "_" + robotPosition[1] + "_" + ori;

		else if (ori == Orientation.SOUTH && (robotPosition[0] - 2) != -1)
			msg = (robotPosition[0] - 2) + "_" + robotPosition[1] + "_" + ori;

		else if (ori == Orientation.EAST && (robotPosition[1] - 2) != -1)
			msg = (robotPosition[0]) + "_" + (robotPosition[1] - 2) + "_" + ori;

		else if (ori == Orientation.WEST && (robotPosition[1] + 2) != 20)
			msg = (robotPosition[0]) + "_" + (robotPosition[1] + 2) + "_" + ori;

		System.out.println("Send to RPI: " + msg);
		if (RobotSystem.isRealRun()) {
			Controller controller = Controller.getInstance();
			PCClient pcClient = controller.getPCClient();
			// pcClient.sendMsgToRPI(msg);

			try {
				Date startDate = new Date();
				Date endDate = new Date();
				int numSeconds = 0;
				pcClient.sendMsgToRPI(msg);
				System.out.println("Sent Take Picture Command");
				String feedback = pcClient.readMessage();
				while (!feedback.equals(Message.DONE)) {
					feedback = pcClient.readMessage();
					System.out.println("Reading Picture Taking Command");
					endDate = new Date();
					numSeconds = (int) ((endDate.getTime() - startDate.getTime()) / 1000);
					System.out.println("Picture Taking Command Timing: " + numSeconds);
					if (((numSeconds % 2) == 0) && numSeconds < 10) {
						pcClient.sendMsgToRPI(msg);
					} else if (numSeconds >= 10)
						break;

				}
			} catch (IOException e) {
				e.printStackTrace();
			}
			System.out.println("Picture Command Completed");
		}

	}

	// Start Exploring
	public void setIsExplored(int[] robotPosition, Orientation ori, boolean hasCalibration) {
		// int count = 0;
		// System.out.println("COUNT VALUE : " + count);
		// while(count == 0) {
		// System.out.println("COUNT VALUE ENTERED : " + count);
		// count++;
		// System.out.println("COUNT VALUE ENTERED ++ : " + count);
		String msgSensorValues = "";
		// System.out.println("SENSOR VALUES ALA" + msgSensorValues);

		int[] leftObsPos = new int[2];

		if (RobotSystem.isRealRun()) {
			try {
				Controller controller = Controller.getInstance();
				PCClient pcClient = controller.getPCClient();

				pcClient.sendMessage(Message.READ_SENSOR_VALUES);
				// if(msgSensorValues.isEmpty()) {
				msgSensorValues = pcClient.readMessage();
				// }

			} catch (IOException e) {
				e.printStackTrace();
			}
		}
		/*
		 * if (RobotSystem.isRealRun()) { try { Controller controller =
		 * Controller.getInstance(); PCClient pcClient = controller.getPCClient();
		 * 
		 * pcClient.sendMessage(Message.READ_SENSOR_VALUES + Message.SEPARATOR);
		 * System.out.println("SENSOR VALUES ALAAAA" + msgSensorValues);
		 * if(msgSensorValues.isEmpty()) { msgSensorValues = pcClient.readMessage(); }
		 * 
		 * } catch (IOException e) { e.printStackTrace(); } }
		 */
		int[] frontSensorPosition = new int[2];
		int[] frontleftSensorPosition = new int[2];
		int[] frontrightSensorPosition = new int[2];
		int[] leftSensorPosition = new int[2];
		int[] rightSensorPosition = new int[2];
		int[] rightSensor2Position = new int[2];
		int numOfClearGrids = 0;

		switch (ori) {
		case NORTH:
			frontSensorPosition[0] = robotPosition[0];
			frontSensorPosition[1] = robotPosition[1];

			frontleftSensorPosition[0] = robotPosition[0] - 1;
			frontleftSensorPosition[1] = robotPosition[1];

			frontrightSensorPosition[0] = robotPosition[0] + 1;
			frontrightSensorPosition[1] = robotPosition[1];

			leftSensorPosition[0] = robotPosition[0];
			leftSensorPosition[1] = robotPosition[1] + 1;

			rightSensorPosition[0] = robotPosition[0];
			rightSensorPosition[1] = robotPosition[1] + 1;

			rightSensor2Position[0] = robotPosition[0];
			rightSensor2Position[1] = robotPosition[1] - 1;

			if (!RobotSystem.isRealRun()) {
				numOfClearGrids = _robot.senseFront(frontSensorPosition, ori);
			} else {
				numOfClearGrids = parseSensorValue(msgSensorValues, SensorPosition.CF);
			}
			if (numOfClearGrids != INVALID_SENSOR_VALUE && numOfClearGrids + frontSensorPosition[1] < Arena.MAP_WIDTH) {

				for (int i = 2; i <= numOfClearGrids; i++) {
					_isExplored[frontSensorPosition[0]][frontSensorPosition[1] + i] = true;
					_mazeRef[frontSensorPosition[0]][frontSensorPosition[1] + i] = IS_EMPTY;

					// weightageRef[frontSensorPosition[0]][frontSensorPosition[1] + i] -= 1;

					// updateWeightageDecrease(i, frontSensorPosition[0], frontSensorPosition[1] +
					// i);
				}
				if (numOfClearGrids < Sensor.SHORT_RANGE
						&& frontSensorPosition[1] + numOfClearGrids + 1 < Arena.MAP_WIDTH) {

					_isExplored[frontSensorPosition[0]][frontSensorPosition[1] + numOfClearGrids + 1] = true;
					_mazeRef[frontSensorPosition[0]][frontSensorPosition[1] + numOfClearGrids + 1] = IS_OBSTACLE;

					/*
					 * _isExplored[frontSensorPosition[0]][frontSensorPosition[1] + numOfClearGrids
					 * + 1] = true;
					 * 
					 * weightageRef[frontSensorPosition[0]][frontSensorPosition[1] + numOfClearGrids
					 * + 1] += 1;
					 * 
					 * if((weightageRef[frontSensorPosition[0]][frontSensorPosition[1] +
					 * numOfClearGrids + 1]) > 0)
					 * _mazeRef[frontSensorPosition[0]][frontSensorPosition[1] + numOfClearGrids +
					 * 1] = IS_OBSTACLE; else
					 * _mazeRef[frontSensorPosition[0]][frontSensorPosition[1] + numOfClearGrids +
					 * 1] = IS_EMPTY;
					 */

					// updateWeightageIncrease(numOfClearGrids, frontSensorPosition[0],
					// frontSensorPosition[1] + numOfClearGrids + 1);
				}
			}
			if (!RobotSystem.isRealRun()) {
				numOfClearGrids = _robot.senseSideFront(frontleftSensorPosition, ori);
			} else {
				numOfClearGrids = parseSensorValue(msgSensorValues, SensorPosition.LF);

			}
			if (numOfClearGrids != INVALID_SENSOR_VALUE
					&& numOfClearGrids + frontleftSensorPosition[1] < Arena.MAP_WIDTH) {

				for (int i = 2; i <= numOfClearGrids; i++) {
					_isExplored[frontleftSensorPosition[0]][frontleftSensorPosition[1] + i] = true;
					_mazeRef[frontleftSensorPosition[0]][frontleftSensorPosition[1] + i] = IS_EMPTY;
				}
				if (numOfClearGrids < Sensor.SHORT_RANGE
						&& frontleftSensorPosition[1] + numOfClearGrids + 1 < Arena.MAP_WIDTH) {
					_isExplored[frontleftSensorPosition[0]][frontleftSensorPosition[1] + numOfClearGrids + 1] = true;
					_mazeRef[frontleftSensorPosition[0]][frontleftSensorPosition[1] + numOfClearGrids
							+ 1] = IS_OBSTACLE;
				}

				/*
				 * for (int i = 2; i <= numOfClearGrids; i++) {
				 * _isExplored[frontleftSensorPosition[0]][frontleftSensorPosition[1] + i] =
				 * true; _mazeRef[frontleftSensorPosition[0]][frontleftSensorPosition[1] + i] =
				 * IS_EMPTY;
				 * 
				 * weightageRef[frontleftSensorPosition[0]][frontleftSensorPosition[1] + i] -=
				 * 1; } if (numOfClearGrids < Sensor.SHORT_RANGE && frontleftSensorPosition[1] +
				 * numOfClearGrids + 1 < Arena.MAP_WIDTH) {
				 * _isExplored[frontleftSensorPosition[0]][frontleftSensorPosition[1] +
				 * numOfClearGrids + 1] = true;
				 * 
				 * weightageRef[frontleftSensorPosition[0]][frontleftSensorPosition[1] +
				 * numOfClearGrids + 1] += 1;
				 * 
				 * if((weightageRef[frontleftSensorPosition[0]][frontleftSensorPosition[1] +
				 * numOfClearGrids + 1]) > 0)
				 * _mazeRef[frontleftSensorPosition[0]][frontleftSensorPosition[1] +
				 * numOfClearGrids + 1] = IS_OBSTACLE; else
				 * _mazeRef[frontleftSensorPosition[0]][frontleftSensorPosition[1] +
				 * numOfClearGrids + 1] = IS_EMPTY;
				 * 
				 * }
				 */
			}
			if (!RobotSystem.isRealRun()) {
				numOfClearGrids = _robot.senseSideFront(frontrightSensorPosition, ori);
			} else {

				numOfClearGrids = parseSensorValue(msgSensorValues, SensorPosition.RF);

			}
			if (numOfClearGrids != INVALID_SENSOR_VALUE
					&& numOfClearGrids + frontrightSensorPosition[1] < Arena.MAP_WIDTH) {

				for (int i = 2; i <= numOfClearGrids; i++) {
					_isExplored[frontrightSensorPosition[0]][frontrightSensorPosition[1] + i] = true;
					_mazeRef[frontrightSensorPosition[0]][frontrightSensorPosition[1] + i] = IS_EMPTY;
				}
				if (numOfClearGrids < Sensor.SHORT_RANGE
						&& frontrightSensorPosition[1] + numOfClearGrids + 1 < Arena.MAP_WIDTH) {
					_isExplored[frontrightSensorPosition[0]][frontrightSensorPosition[1] + numOfClearGrids + 1] = true;
					_mazeRef[frontrightSensorPosition[0]][frontrightSensorPosition[1] + numOfClearGrids
							+ 1] = IS_OBSTACLE;
				}

				/*
				 * for (int i = 2; i <= numOfClearGrids; i++) {
				 * _isExplored[frontrightSensorPosition[0]][frontrightSensorPosition[1] + i] =
				 * true; _mazeRef[frontrightSensorPosition[0]][frontrightSensorPosition[1] + i]
				 * = IS_EMPTY;
				 * 
				 * weightageRef[frontrightSensorPosition[0]][frontrightSensorPosition[1] + i] -=
				 * 1; } if (numOfClearGrids < Sensor.SHORT_RANGE && frontrightSensorPosition[1]
				 * + numOfClearGrids + 1 < Arena.MAP_WIDTH) {
				 * _isExplored[frontrightSensorPosition[0]][frontrightSensorPosition[1] +
				 * numOfClearGrids + 1] = true;
				 * 
				 * weightageRef[frontrightSensorPosition[0]][frontrightSensorPosition[1] +
				 * numOfClearGrids + 1] += 1;
				 * 
				 * if((weightageRef[frontrightSensorPosition[0]][frontrightSensorPosition[1] +
				 * numOfClearGrids + 1]) > 0)
				 * _mazeRef[frontrightSensorPosition[0]][frontrightSensorPosition[1] +
				 * numOfClearGrids + 1] = IS_OBSTACLE; else
				 * _mazeRef[frontrightSensorPosition[0]][frontrightSensorPosition[1] +
				 * numOfClearGrids + 1] = IS_EMPTY;
				 * 
				 * }
				 */
			}
			if (!RobotSystem.isRealRun()) {
				numOfClearGrids = _robot.senseLeft(leftSensorPosition, ori);
			} else {
				numOfClearGrids = parseSensorValue(msgSensorValues, SensorPosition.L);
			}
			if (numOfClearGrids != INVALID_SENSOR_VALUE && leftSensorPosition[0] - numOfClearGrids >= 0) {

				for (int i = 2; i <= numOfClearGrids; i++) {
					_isExplored[leftSensorPosition[0] - i][leftSensorPosition[1]] = true;
					_mazeRef[leftSensorPosition[0] - i][leftSensorPosition[1]] = IS_EMPTY;
				}
				if (numOfClearGrids < Sensor.LONG_RANGE && leftSensorPosition[0] - numOfClearGrids - 1 >= 0) {
					_isExplored[leftSensorPosition[0] - numOfClearGrids - 1][leftSensorPosition[1]] = true;
					_mazeRef[leftSensorPosition[0] - numOfClearGrids - 1][leftSensorPosition[1]] = IS_OBSTACLE;
				}

				/*
				 * for (int i = 2; i <= numOfClearGrids; i++) {
				 * _isExplored[leftSensorPosition[0] - i][leftSensorPosition[1]] = true;
				 * _mazeRef[leftSensorPosition[0] - i][leftSensorPosition[1]] = IS_EMPTY;
				 * weightageRef[leftSensorPosition[0] - i][leftSensorPosition[1]] -= 1; } if
				 * (numOfClearGrids < Sensor.LONG_RANGE && leftSensorPosition[0] -
				 * numOfClearGrids - 1 >= 0) { _isExplored[leftSensorPosition[0] -
				 * numOfClearGrids - 1][leftSensorPosition[1]] = true;
				 * weightageRef[leftSensorPosition[0] - numOfClearGrids -
				 * 1][leftSensorPosition[1]] += 1;
				 * 
				 * if((weightageRef[leftSensorPosition[0] - numOfClearGrids -
				 * 1][leftSensorPosition[1]]) > 0) { _mazeRef[leftSensorPosition[0] -
				 * numOfClearGrids - 1][leftSensorPosition[1]] = IS_OBSTACLE; leftGotObstacle =
				 * true; leftObsPos[0] = leftSensorPosition[0] - numOfClearGrids - 1;
				 * leftObsPos[1] = leftSensorPosition[1]; } else {
				 * _mazeRef[leftSensorPosition[0] - numOfClearGrids - 1][leftSensorPosition[1]]
				 * = IS_EMPTY; leftTurnCounter = 0; }
				 * 
				 * }
				 */
			}

			if (!RobotSystem.isRealRun()) {
				numOfClearGrids = _robot.senseRight(rightSensorPosition, ori);
			} else {
				numOfClearGrids = parseSensorValue(msgSensorValues, SensorPosition.R);
			}
			if (numOfClearGrids != INVALID_SENSOR_VALUE
					&& rightSensorPosition[0] + numOfClearGrids < Arena.MAP_LENGTH) {

				for (int i = 2; i <= numOfClearGrids; i++) {
					_isExplored[rightSensorPosition[0] + i][rightSensorPosition[1]] = true;
					_mazeRef[rightSensorPosition[0] + i][rightSensorPosition[1]] = IS_EMPTY;
				}
				if (numOfClearGrids < Sensor.SHORT_RANGE
						&& rightSensorPosition[0] + numOfClearGrids + 1 < Arena.MAP_LENGTH) {
					_isExplored[rightSensorPosition[0] + numOfClearGrids + 1][rightSensorPosition[1]] = true;
					_mazeRef[rightSensorPosition[0] + numOfClearGrids + 1][rightSensorPosition[1]] = IS_OBSTACLE;
				}

				/*
				 * for (int i = 2; i <= numOfClearGrids; i++) {
				 * _isExplored[rightSensorPosition[0] + i][rightSensorPosition[1]] = true;
				 * _mazeRef[rightSensorPosition[0] + i][rightSensorPosition[1]] = IS_EMPTY;
				 * weightageRef[rightSensorPosition[0] + i][rightSensorPosition[1]] -= 1; } if
				 * (numOfClearGrids < Sensor.SHORT_RANGE && rightSensorPosition[0] +
				 * numOfClearGrids + 1 < Arena.MAP_LENGTH) { _isExplored[rightSensorPosition[0]
				 * + numOfClearGrids + 1][rightSensorPosition[1]] = true;
				 * weightageRef[rightSensorPosition[0] + numOfClearGrids +
				 * 1][rightSensorPosition[1]] += 1;
				 * 
				 * if((weightageRef[rightSensorPosition[0] + numOfClearGrids +
				 * 1][rightSensorPosition[1]]) > 0) _mazeRef[rightSensorPosition[0] +
				 * numOfClearGrids + 1][rightSensorPosition[1]] = IS_OBSTACLE; else
				 * _mazeRef[rightSensorPosition[0] + numOfClearGrids +
				 * 1][rightSensorPosition[1]] = IS_EMPTY;
				 * 
				 * }
				 */
			}
			if (!RobotSystem.isRealRun()) {
				numOfClearGrids = _robot.senseRight(rightSensor2Position, ori);
			} else {
				numOfClearGrids = parseSensorValue(msgSensorValues, SensorPosition.R2);
			}
			if (numOfClearGrids != INVALID_SENSOR_VALUE
					&& rightSensor2Position[0] + numOfClearGrids < Arena.MAP_LENGTH) {

				for (int i = 2; i <= numOfClearGrids; i++) {
					_isExplored[rightSensor2Position[0] + i][rightSensor2Position[1]] = true;
					_mazeRef[rightSensor2Position[0] + i][rightSensor2Position[1]] = IS_EMPTY;
				}
				if (numOfClearGrids < Sensor.SHORT_RANGE
						&& rightSensor2Position[0] + numOfClearGrids + 1 < Arena.MAP_LENGTH) {
					_isExplored[rightSensor2Position[0] + numOfClearGrids + 1][rightSensor2Position[1]] = true;
					_mazeRef[rightSensor2Position[0] + numOfClearGrids + 1][rightSensor2Position[1]] = IS_OBSTACLE;
				}

				/*
				 * for (int i = 2; i <= numOfClearGrids; i++) {
				 * _isExplored[rightSensor2Position[0] + i][rightSensor2Position[1]] = true;
				 * _mazeRef[rightSensor2Position[0] + i][rightSensor2Position[1]] = IS_EMPTY;
				 * weightageRef[rightSensor2Position[0] + i][rightSensor2Position[1]] -= 1; } if
				 * (numOfClearGrids < Sensor.SHORT_RANGE && rightSensor2Position[0] +
				 * numOfClearGrids + 1 < Arena.MAP_LENGTH) { _isExplored[rightSensor2Position[0]
				 * + numOfClearGrids + 1][rightSensor2Position[1]] = true;
				 * 
				 * weightageRef[rightSensor2Position[0] + numOfClearGrids +
				 * 1][rightSensor2Position[1]] += 1;
				 * 
				 * if((weightageRef[rightSensor2Position[0] + numOfClearGrids +
				 * 1][rightSensor2Position[1]]) > 0) _mazeRef[rightSensor2Position[0] +
				 * numOfClearGrids + 1][rightSensor2Position[1]] = IS_OBSTACLE; else
				 * _mazeRef[rightSensor2Position[0] + numOfClearGrids +
				 * 1][rightSensor2Position[1]] = IS_EMPTY;
				 * 
				 * }
				 */
			}
			break;

		case SOUTH:
			System.out.println("SOUTH ");
			frontSensorPosition[0] = robotPosition[0];
			frontSensorPosition[1] = robotPosition[1];

			frontleftSensorPosition[0] = robotPosition[0] + 1;
			frontleftSensorPosition[1] = robotPosition[1];

			frontrightSensorPosition[0] = robotPosition[0] - 1;
			frontrightSensorPosition[1] = robotPosition[1];

			leftSensorPosition[0] = robotPosition[0];
			leftSensorPosition[1] = robotPosition[1] - 1;

			rightSensorPosition[0] = robotPosition[0];
			rightSensorPosition[1] = robotPosition[1] - 1;

			rightSensor2Position[0] = robotPosition[0];
			rightSensor2Position[1] = robotPosition[1] + 1;

			if (!RobotSystem.isRealRun()) {
				numOfClearGrids = _robot.senseFront(frontSensorPosition, ori);
			} else {
				numOfClearGrids = parseSensorValue(msgSensorValues, SensorPosition.CF);
			}
			if (numOfClearGrids != INVALID_SENSOR_VALUE && frontSensorPosition[1] - numOfClearGrids >= 0) {

				for (int i = 2; i <= numOfClearGrids; i++) {
					_isExplored[frontSensorPosition[0]][frontSensorPosition[1] - i] = true;
					_mazeRef[frontSensorPosition[0]][frontSensorPosition[1] - i] = IS_EMPTY;
				}
				if (numOfClearGrids < Sensor.SHORT_RANGE && frontSensorPosition[1] - numOfClearGrids - 1 >= 0) {
					_isExplored[frontSensorPosition[0]][frontSensorPosition[1] - numOfClearGrids - 1] = true;
					_mazeRef[frontSensorPosition[0]][frontSensorPosition[1] - numOfClearGrids - 1] = IS_OBSTACLE;
				}

				/*
				 * for (int i = 2; i <= numOfClearGrids; i++) {
				 * _isExplored[frontSensorPosition[0]][frontSensorPosition[1] - i] = true;
				 * _mazeRef[frontSensorPosition[0]][frontSensorPosition[1] - i] = IS_EMPTY;
				 * weightageRef[frontSensorPosition[0]][frontSensorPosition[1] - i] -= 1; } if
				 * (numOfClearGrids < Sensor.SHORT_RANGE && frontSensorPosition[1] -
				 * numOfClearGrids - 1 >= 0) {
				 * _isExplored[frontSensorPosition[0]][frontSensorPosition[1] - numOfClearGrids
				 * - 1] = true;
				 * 
				 * weightageRef[frontSensorPosition[0]][frontSensorPosition[1] - numOfClearGrids
				 * - 1] += 1;
				 * 
				 * if((weightageRef[frontSensorPosition[0]][frontSensorPosition[1] -
				 * numOfClearGrids - 1]) > 0)
				 * _mazeRef[frontSensorPosition[0]][frontSensorPosition[1] - numOfClearGrids -
				 * 1] = IS_OBSTACLE; else
				 * _mazeRef[frontSensorPosition[0]][frontSensorPosition[1] - numOfClearGrids -
				 * 1] = IS_EMPTY;
				 * 
				 * }
				 */

			}
			if (!RobotSystem.isRealRun()) {
				numOfClearGrids = _robot.senseSideFront(frontleftSensorPosition, ori);
			} else {
				numOfClearGrids = parseSensorValue(msgSensorValues, SensorPosition.LF);
			}
			if (numOfClearGrids != INVALID_SENSOR_VALUE && frontleftSensorPosition[1] - numOfClearGrids >= 0) {

				for (int i = 2; i <= numOfClearGrids; i++) {
					_isExplored[frontleftSensorPosition[0]][frontleftSensorPosition[1] - i] = true;
					_mazeRef[frontleftSensorPosition[0]][frontleftSensorPosition[1] - i] = IS_EMPTY;
				}
				if (numOfClearGrids < Sensor.SHORT_RANGE && frontleftSensorPosition[1] - numOfClearGrids - 1 >= 0) {
					_isExplored[frontleftSensorPosition[0]][frontleftSensorPosition[1] - numOfClearGrids - 1] = true;
					_mazeRef[frontleftSensorPosition[0]][frontleftSensorPosition[1] - numOfClearGrids
							- 1] = IS_OBSTACLE;
				}

				/*
				 * for (int i = 2; i <= numOfClearGrids; i++) {
				 * _isExplored[frontleftSensorPosition[0]][frontleftSensorPosition[1] - i] =
				 * true; _mazeRef[frontleftSensorPosition[0]][frontleftSensorPosition[1] - i] =
				 * IS_EMPTY; weightageRef[frontleftSensorPosition[0]][frontleftSensorPosition[1]
				 * - i] -= 1; } if (numOfClearGrids < Sensor.SHORT_RANGE &&
				 * frontleftSensorPosition[1] - numOfClearGrids - 1 >= 0) {
				 * _isExplored[frontleftSensorPosition[0]][frontleftSensorPosition[1] -
				 * numOfClearGrids - 1] = true;
				 * 
				 * weightageRef[frontleftSensorPosition[0]][frontleftSensorPosition[1] -
				 * numOfClearGrids - 1] += 1;
				 * 
				 * if((weightageRef[frontleftSensorPosition[0]][frontleftSensorPosition[1] -
				 * numOfClearGrids - 1] > 0))
				 * _mazeRef[frontleftSensorPosition[0]][frontleftSensorPosition[1] -
				 * numOfClearGrids - 1] = IS_OBSTACLE; else
				 * _mazeRef[frontleftSensorPosition[0]][frontleftSensorPosition[1] -
				 * numOfClearGrids - 1] = IS_EMPTY; }
				 */

			}
			if (!RobotSystem.isRealRun()) {
				numOfClearGrids = _robot.senseSideFront(frontrightSensorPosition, ori);
			} else {
				numOfClearGrids = parseSensorValue(msgSensorValues, SensorPosition.RF);
			}
			if (numOfClearGrids != INVALID_SENSOR_VALUE && frontrightSensorPosition[1] - numOfClearGrids >= 0) {

				for (int i = 2; i <= numOfClearGrids; i++) {
					_isExplored[frontrightSensorPosition[0]][frontrightSensorPosition[1] - i] = true;
					_mazeRef[frontrightSensorPosition[0]][frontrightSensorPosition[1] - i] = IS_EMPTY;
				}
				if (numOfClearGrids < Sensor.SHORT_RANGE && frontrightSensorPosition[1] - numOfClearGrids - 1 >= 0) {
					_isExplored[frontrightSensorPosition[0]][frontrightSensorPosition[1] - numOfClearGrids - 1] = true;
					_mazeRef[frontrightSensorPosition[0]][frontrightSensorPosition[1] - numOfClearGrids
							- 1] = IS_OBSTACLE;
				}

				/*
				 * for (int i = 2; i <= numOfClearGrids; i++) {
				 * _isExplored[frontrightSensorPosition[0]][frontrightSensorPosition[1] - i] =
				 * true; _mazeRef[frontrightSensorPosition[0]][frontrightSensorPosition[1] - i]
				 * = IS_EMPTY;
				 * weightageRef[frontrightSensorPosition[0]][frontrightSensorPosition[1] - i] -=
				 * 1; } if (numOfClearGrids < Sensor.SHORT_RANGE && frontrightSensorPosition[1]
				 * - numOfClearGrids - 1 >= 0) {
				 * _isExplored[frontrightSensorPosition[0]][frontrightSensorPosition[1] -
				 * numOfClearGrids - 1] = true;
				 * 
				 * weightageRef[frontrightSensorPosition[0]][frontrightSensorPosition[1] -
				 * numOfClearGrids - 1] += 1;
				 * 
				 * if((weightageRef[frontrightSensorPosition[0]][frontrightSensorPosition[1] -
				 * numOfClearGrids - 1] > 0))
				 * _mazeRef[frontrightSensorPosition[0]][frontrightSensorPosition[1] -
				 * numOfClearGrids - 1] = IS_OBSTACLE; else
				 * _mazeRef[frontrightSensorPosition[0]][frontrightSensorPosition[1] -
				 * numOfClearGrids - 1] = IS_EMPTY;
				 * 
				 * }
				 */
			}
			if (!RobotSystem.isRealRun()) {
				numOfClearGrids = _robot.senseLeft(leftSensorPosition, ori);
			} else {
				numOfClearGrids = parseSensorValue(msgSensorValues, SensorPosition.L);
			}
			if (numOfClearGrids != INVALID_SENSOR_VALUE && leftSensorPosition[0] + numOfClearGrids < Arena.MAP_LENGTH) {

				for (int i = 2; i <= numOfClearGrids; i++) {
					_isExplored[leftSensorPosition[0] + i][leftSensorPosition[1]] = true;
					_mazeRef[leftSensorPosition[0] + i][leftSensorPosition[1]] = IS_EMPTY;
				}
				if (numOfClearGrids < Sensor.LONG_RANGE
						&& leftSensorPosition[0] + numOfClearGrids + 1 < Arena.MAP_LENGTH) {
					_isExplored[leftSensorPosition[0] + numOfClearGrids + 1][leftSensorPosition[1]] = true;
					_mazeRef[leftSensorPosition[0] + numOfClearGrids + 1][leftSensorPosition[1]] = IS_OBSTACLE;
				}

				/*
				 * for (int i = 2; i <= numOfClearGrids; i++) {
				 * _isExplored[leftSensorPosition[0] + i][leftSensorPosition[1]] = true;
				 * _mazeRef[leftSensorPosition[0] + i][leftSensorPosition[1]] = IS_EMPTY;
				 * weightageRef[leftSensorPosition[0] + i][leftSensorPosition[1]] -= 1; } if
				 * (numOfClearGrids < Sensor.LONG_RANGE && leftSensorPosition[0] +
				 * numOfClearGrids + 1 < Arena.MAP_LENGTH) { _isExplored[leftSensorPosition[0] +
				 * numOfClearGrids + 1][leftSensorPosition[1]] = true;
				 * 
				 * weightageRef[leftSensorPosition[0] + numOfClearGrids +
				 * 1][leftSensorPosition[1]] += 1;
				 * 
				 * if((weightageRef[leftSensorPosition[0] + numOfClearGrids +
				 * 1][leftSensorPosition[1]] > 0)) { _mazeRef[leftSensorPosition[0] +
				 * numOfClearGrids + 1][leftSensorPosition[1]] = IS_OBSTACLE; leftGotObstacle =
				 * true; leftObsPos[0] = leftSensorPosition[0] + numOfClearGrids + 1;
				 * leftObsPos[1] = leftSensorPosition[1]; } else {
				 * _mazeRef[leftSensorPosition[0] + numOfClearGrids + 1][leftSensorPosition[1]]
				 * = IS_EMPTY; leftTurnCounter = 0; } }
				 */

			}
			if (!RobotSystem.isRealRun()) {
				numOfClearGrids = _robot.senseRight(rightSensorPosition, ori);
			} else {
				numOfClearGrids = parseSensorValue(msgSensorValues, SensorPosition.R);
			}
			if (numOfClearGrids != INVALID_SENSOR_VALUE && rightSensorPosition[0] - numOfClearGrids >= 0) {

				for (int i = 2; i <= numOfClearGrids; i++) {
					_isExplored[rightSensorPosition[0] - i][rightSensorPosition[1]] = true;
					_mazeRef[rightSensorPosition[0] - i][rightSensorPosition[1]] = IS_EMPTY;
				}
				if (numOfClearGrids < Sensor.SHORT_RANGE && rightSensorPosition[0] - numOfClearGrids - 1 >= 0) {
					_isExplored[rightSensorPosition[0] - numOfClearGrids - 1][rightSensorPosition[1]] = true;
					_mazeRef[rightSensorPosition[0] - numOfClearGrids - 1][rightSensorPosition[1]] = IS_OBSTACLE;
				}

				/*
				 * for (int i = 2; i <= numOfClearGrids; i++) {
				 * _isExplored[rightSensorPosition[0] - i][rightSensorPosition[1]] = true;
				 * _mazeRef[rightSensorPosition[0] - i][rightSensorPosition[1]] = IS_EMPTY;
				 * weightageRef[rightSensorPosition[0] - i][rightSensorPosition[1]] -= 1; } if
				 * (numOfClearGrids < Sensor.SHORT_RANGE && rightSensorPosition[0] -
				 * numOfClearGrids - 1 >= 0) { _isExplored[rightSensorPosition[0] -
				 * numOfClearGrids - 1][rightSensorPosition[1]] = true;
				 * 
				 * weightageRef[rightSensorPosition[0] - numOfClearGrids -
				 * 1][rightSensorPosition[1]] += 1;
				 * 
				 * if((weightageRef[rightSensorPosition[0] - numOfClearGrids -
				 * 1][rightSensorPosition[1]] > 0)) _mazeRef[rightSensorPosition[0] -
				 * numOfClearGrids - 1][rightSensorPosition[1]] = IS_OBSTACLE; else
				 * _mazeRef[rightSensorPosition[0] - numOfClearGrids -
				 * 1][rightSensorPosition[1]] = IS_EMPTY;
				 * 
				 * }
				 */
			}
			if (!RobotSystem.isRealRun()) {
				numOfClearGrids = _robot.senseRight(rightSensor2Position, ori);
			} else {
				numOfClearGrids = parseSensorValue(msgSensorValues, SensorPosition.R2);
			}
			if (numOfClearGrids != INVALID_SENSOR_VALUE && rightSensor2Position[0] - numOfClearGrids >= 0) {

				for (int i = 2; i <= numOfClearGrids; i++) {
					_isExplored[rightSensor2Position[0] - i][rightSensor2Position[1]] = true;
					_mazeRef[rightSensor2Position[0] - i][rightSensor2Position[1]] = IS_EMPTY;
				}
				if (numOfClearGrids < Sensor.SHORT_RANGE && rightSensor2Position[0] - numOfClearGrids - 1 >= 0) {
					_isExplored[rightSensor2Position[0] - numOfClearGrids - 1][rightSensor2Position[1]] = true;
					_mazeRef[rightSensor2Position[0] - numOfClearGrids - 1][rightSensor2Position[1]] = IS_OBSTACLE;
				}

				/*
				 * for (int i = 2; i <= numOfClearGrids; i++) {
				 * _isExplored[rightSensor2Position[0] - i][rightSensor2Position[1]] = true;
				 * _mazeRef[rightSensor2Position[0] - i][rightSensor2Position[1]] = IS_EMPTY;
				 * weightageRef[rightSensor2Position[0] - i][rightSensor2Position[1]] -= 1; } if
				 * (numOfClearGrids < Sensor.SHORT_RANGE && rightSensor2Position[0] -
				 * numOfClearGrids - 1 >= 0) { _isExplored[rightSensor2Position[0] -
				 * numOfClearGrids - 1][rightSensor2Position[1]] = true;
				 * 
				 * weightageRef[rightSensor2Position[0] - numOfClearGrids -
				 * 1][rightSensor2Position[1]] += 1;
				 * 
				 * if((weightageRef[rightSensor2Position[0] - numOfClearGrids -
				 * 1][rightSensor2Position[1]] > 0)) _mazeRef[rightSensor2Position[0] -
				 * numOfClearGrids - 1][rightSensor2Position[1]] = IS_OBSTACLE; else
				 * _mazeRef[rightSensor2Position[0] - numOfClearGrids -
				 * 1][rightSensor2Position[1]] = IS_EMPTY;
				 * 
				 * }
				 */
			}
			break;

		case EAST:
			System.out.println("EAST ");
			frontSensorPosition[0] = robotPosition[0];
			frontSensorPosition[1] = robotPosition[1];
			frontleftSensorPosition[0] = robotPosition[0];
			frontleftSensorPosition[1] = robotPosition[1] + 1;
			frontrightSensorPosition[0] = robotPosition[0];
			frontrightSensorPosition[1] = robotPosition[1] - 1;
			leftSensorPosition[0] = robotPosition[0] + 1;
			leftSensorPosition[1] = robotPosition[1];
			rightSensorPosition[0] = robotPosition[0] + 1;
			rightSensorPosition[1] = robotPosition[1];
			rightSensor2Position[0] = robotPosition[0] - 1;
			rightSensor2Position[1] = robotPosition[1];

			if (!RobotSystem.isRealRun()) {
				numOfClearGrids = _robot.senseFront(frontSensorPosition, ori);
			} else {
				// System.out.println(numOfClearGrids + "BEFORE CF NUMBER OF GRIDS");
				// System.out.println(msgSensorValues + "BEFORE CF SENSOR VALUES");
				numOfClearGrids = parseSensorValue(msgSensorValues, SensorPosition.CF);
				// System.out.println(numOfClearGrids + "AFTER CF NUMBER OF GRIDS");
				// System.out.println(msgSensorValues + "AFTER CF SENSOR VALUES");
			}
			if (numOfClearGrids != INVALID_SENSOR_VALUE
					&& frontSensorPosition[0] + numOfClearGrids < Arena.MAP_LENGTH) {

				for (int i = 2; i <= numOfClearGrids; i++) {
					_isExplored[frontSensorPosition[0] + i][frontSensorPosition[1]] = true;
					_mazeRef[frontSensorPosition[0] + i][frontSensorPosition[1]] = IS_EMPTY;
				}
				if (numOfClearGrids < Sensor.SHORT_RANGE
						&& frontSensorPosition[0] + numOfClearGrids + 1 < Arena.MAP_LENGTH) {
					_isExplored[frontSensorPosition[0] + numOfClearGrids + 1][frontSensorPosition[1]] = true;
					_mazeRef[frontSensorPosition[0] + numOfClearGrids + 1][frontSensorPosition[1]] = IS_OBSTACLE;
				}

				/*
				 * for (int i = 2; i <= numOfClearGrids; i++) {
				 * _isExplored[frontSensorPosition[0] + i][frontSensorPosition[1]] = true;
				 * _mazeRef[frontSensorPosition[0] + i][frontSensorPosition[1]] = IS_EMPTY;
				 * weightageRef[frontSensorPosition[0] + i][frontSensorPosition[1]] -= 1; } if
				 * (numOfClearGrids < Sensor.SHORT_RANGE && frontSensorPosition[0] +
				 * numOfClearGrids + 1 < Arena.MAP_LENGTH) { _isExplored[frontSensorPosition[0]
				 * + numOfClearGrids + 1][frontSensorPosition[1]] = true;
				 * 
				 * weightageRef [frontSensorPosition[0] + numOfClearGrids +
				 * 1][frontSensorPosition[1]] += 1;
				 * 
				 * if((weightageRef [frontSensorPosition[0] + numOfClearGrids +
				 * 1][frontSensorPosition[1]] > 0)) _mazeRef [frontSensorPosition[0] +
				 * numOfClearGrids + 1][frontSensorPosition[1]] = IS_OBSTACLE; else _mazeRef
				 * [frontSensorPosition[0] + numOfClearGrids + 1][frontSensorPosition[1]] =
				 * IS_EMPTY; }
				 */

			}
			if (!RobotSystem.isRealRun()) {
				numOfClearGrids = _robot.senseSideFront(frontleftSensorPosition, ori);
			} else {
				// System.out.println(numOfClearGrids + "BEFORE LF NUMBER OF GRIDS");
				// System.out.println(msgSensorValues + "BEFIRE LF SENSOR VALUES");
				numOfClearGrids = parseSensorValue(msgSensorValues, SensorPosition.LF);
				// System.out.println(numOfClearGrids + "AFTER LF NUMBER OF GRIDS");
				// System.out.println(msgSensorValues + "AFTER LF SENSOR VALUES");
			}
			if (numOfClearGrids != INVALID_SENSOR_VALUE
					&& frontleftSensorPosition[0] + numOfClearGrids < Arena.MAP_LENGTH) {

				for (int i = 2; i <= numOfClearGrids; i++) {
					_isExplored[frontleftSensorPosition[0] + i][frontleftSensorPosition[1]] = true;
					_mazeRef[frontleftSensorPosition[0] + i][frontleftSensorPosition[1]] = IS_EMPTY;
				}
				if (numOfClearGrids < Sensor.SHORT_RANGE
						&& frontleftSensorPosition[0] + numOfClearGrids + 1 < Arena.MAP_LENGTH) {
					_isExplored[frontleftSensorPosition[0] + numOfClearGrids + 1][frontleftSensorPosition[1]] = true;
					_mazeRef[frontleftSensorPosition[0] + numOfClearGrids
							+ 1][frontleftSensorPosition[1]] = IS_OBSTACLE;
				}

				/*
				 * for (int i = 2; i <= numOfClearGrids; i++) {
				 * _isExplored[frontleftSensorPosition[0] + i][frontleftSensorPosition[1]] =
				 * true; _mazeRef[frontleftSensorPosition[0] + i][frontleftSensorPosition[1]] =
				 * IS_EMPTY; weightageRef[frontleftSensorPosition[0] +
				 * i][frontleftSensorPosition[1]] -= 1; } if (numOfClearGrids <
				 * Sensor.SHORT_RANGE && frontleftSensorPosition[0] + numOfClearGrids + 1 <
				 * Arena.MAP_LENGTH) { _isExplored[frontleftSensorPosition[0] + numOfClearGrids
				 * + 1][frontleftSensorPosition[1]] = true;
				 * 
				 * weightageRef [frontleftSensorPosition[0] + numOfClearGrids +
				 * 1][frontleftSensorPosition[1]] += 1;
				 * 
				 * if((weightageRef [frontleftSensorPosition[0] + numOfClearGrids +
				 * 1][frontleftSensorPosition[1]] > 0)) _mazeRef [frontleftSensorPosition[0] +
				 * numOfClearGrids + 1][frontleftSensorPosition[1]] = IS_OBSTACLE; else _mazeRef
				 * [frontleftSensorPosition[0] + numOfClearGrids +
				 * 1][frontleftSensorPosition[1]] = IS_EMPTY; }
				 */

			}
			if (!RobotSystem.isRealRun()) {
				numOfClearGrids = _robot.senseSideFront(frontrightSensorPosition, ori);
			} else {
				// System.out.println(numOfClearGrids + "BEFORE RF NUMBER OF GRIDS");
				// System.out.println(msgSensorValues + "BEFIRE RF SENSOR VALUES");
				numOfClearGrids = parseSensorValue(msgSensorValues, SensorPosition.RF);
				// System.out.println(numOfClearGrids + "AFTER RF NUMBER OF GRIDS");
				// System.out.println(msgSensorValues + "AFTER RF SENSOR VALUES");
			}
			if (numOfClearGrids != INVALID_SENSOR_VALUE
					&& frontrightSensorPosition[0] + numOfClearGrids < Arena.MAP_LENGTH) {

				for (int i = 2; i <= numOfClearGrids; i++) {
					_isExplored[frontrightSensorPosition[0] + i][frontrightSensorPosition[1]] = true;
					_mazeRef[frontrightSensorPosition[0] + i][frontrightSensorPosition[1]] = IS_EMPTY;
				}
				if (numOfClearGrids < Sensor.SHORT_RANGE
						&& frontrightSensorPosition[0] + numOfClearGrids + 1 < Arena.MAP_LENGTH) {
					_isExplored[frontrightSensorPosition[0] + numOfClearGrids + 1][frontrightSensorPosition[1]] = true;
					_mazeRef[frontrightSensorPosition[0] + numOfClearGrids
							+ 1][frontrightSensorPosition[1]] = IS_OBSTACLE;
				}

				/*
				 * for (int i = 2; i <= numOfClearGrids; i++) {
				 * _isExplored[frontrightSensorPosition[0] + i][frontrightSensorPosition[1]] =
				 * true; _mazeRef [frontrightSensorPosition[0] +
				 * i][frontrightSensorPosition[1]]= IS_EMPTY; weightageRef
				 * [frontrightSensorPosition[0] + i][frontrightSensorPosition[1]] -= 1; } if
				 * (numOfClearGrids < Sensor.SHORT_RANGE && frontrightSensorPosition[0] +
				 * numOfClearGrids + 1 < Arena.MAP_LENGTH) {
				 * _isExplored[frontrightSensorPosition[0] + numOfClearGrids +
				 * 1][frontrightSensorPosition[1]] = true;
				 * 
				 * weightageRef [frontrightSensorPosition[0] + numOfClearGrids +
				 * 1][frontrightSensorPosition[1]] += 1; if((weightageRef
				 * [frontrightSensorPosition[0] + numOfClearGrids +
				 * 1][frontrightSensorPosition[1]] > 0)) _mazeRef [frontrightSensorPosition[0] +
				 * numOfClearGrids + 1][frontrightSensorPosition[1]] = IS_OBSTACLE; else
				 * _mazeRef [frontrightSensorPosition[0] + numOfClearGrids +
				 * 1][frontrightSensorPosition[1]] = IS_EMPTY; }
				 */

			}
			if (!RobotSystem.isRealRun()) {
				numOfClearGrids = _robot.senseLeft(leftSensorPosition, ori);
			} else {
				// System.out.println(numOfClearGrids + "BEFORE L NUMBER OF GRIDS");
				// System.out.println(msgSensorValues + "BEFORE L SENSOR VALUES");
				numOfClearGrids = parseSensorValue(msgSensorValues, SensorPosition.L);
				// System.out.println(numOfClearGrids + "AFTER L NUMBER OF GRIDS");
				// System.out.println(msgSensorValues + "AFTER L SENSOR VALUES");
			}
			if (numOfClearGrids != INVALID_SENSOR_VALUE && leftSensorPosition[1] + numOfClearGrids < Arena.MAP_WIDTH) {

				for (int i = 2; i <= numOfClearGrids; i++) {
					_isExplored[leftSensorPosition[0]][leftSensorPosition[1] + i] = true;
					_mazeRef[leftSensorPosition[0]][leftSensorPosition[1] + i] = IS_EMPTY;
				}
				if (numOfClearGrids < Sensor.LONG_RANGE
						&& leftSensorPosition[1] + numOfClearGrids + 1 < Arena.MAP_WIDTH) {
					_isExplored[leftSensorPosition[0]][leftSensorPosition[1] + numOfClearGrids + 1] = true;
					_mazeRef[leftSensorPosition[0]][leftSensorPosition[1] + numOfClearGrids + 1] = IS_OBSTACLE;
				}

				/*
				 * for (int i = 2; i <= numOfClearGrids; i++) {
				 * _isExplored[leftSensorPosition[0]][leftSensorPosition[1] + i] = true;
				 * _mazeRef [leftSensorPosition[0]][leftSensorPosition[1] + i]= IS_EMPTY;
				 * weightageRef [leftSensorPosition[0]][leftSensorPosition[1] + i] -= 1; } if
				 * (numOfClearGrids < Sensor.LONG_RANGE && leftSensorPosition[1] +
				 * numOfClearGrids + 1 < Arena.MAP_WIDTH) {
				 * _isExplored[leftSensorPosition[0]][leftSensorPosition[1] + numOfClearGrids +
				 * 1] = true; weightageRef[leftSensorPosition[0]][leftSensorPosition[1] +
				 * numOfClearGrids + 1] += 1;
				 * 
				 * if((weightageRef [leftSensorPosition[0]][leftSensorPosition[1] +
				 * numOfClearGrids + 1] > 0)) {
				 * _mazeRef[leftSensorPosition[0]][leftSensorPosition[1] + numOfClearGrids + 1]
				 * = IS_OBSTACLE; leftGotObstacle = true; leftObsPos[0] = leftSensorPosition[0];
				 * leftObsPos[1] = leftSensorPosition[1] + numOfClearGrids + 1; } else {
				 * _mazeRef[leftSensorPosition[0]][leftSensorPosition[1] + numOfClearGrids + 1]
				 * = IS_EMPTY; leftTurnCounter = 0; } }
				 */
			}
			if (!RobotSystem.isRealRun()) {
				numOfClearGrids = _robot.senseRight(rightSensorPosition, ori);
			} else {
				// System.out.println(numOfClearGrids + "BEFORE R NUMBER OF GRIDS");
				// System.out.println(msgSensorValues + "BEFORE R SENSOR VALUES");
				numOfClearGrids = parseSensorValue(msgSensorValues, SensorPosition.R);
				// System.out.println(numOfClearGrids + "AFTER R NUMBER OF GRIDS");
				// System.out.println(msgSensorValues + "AFTER R SENSOR VALUES");
			}
			if (numOfClearGrids != INVALID_SENSOR_VALUE && rightSensorPosition[1] - numOfClearGrids >= 0) {
				for (int i = 2; i <= numOfClearGrids; i++) {
					_isExplored[rightSensorPosition[0]][rightSensorPosition[1] - i] = true;
					_mazeRef[rightSensorPosition[0]][rightSensorPosition[1] - i] = IS_EMPTY;
				}
				if (numOfClearGrids < Sensor.SHORT_RANGE && rightSensorPosition[1] - numOfClearGrids - 1 >= 0) {
					_isExplored[rightSensorPosition[0]][rightSensorPosition[1] - numOfClearGrids - 1] = true;
					_mazeRef[rightSensorPosition[0]][rightSensorPosition[1] - numOfClearGrids - 1] = IS_OBSTACLE;
				}

				/*
				 * for (int i = 2; i <= numOfClearGrids; i++) {
				 * _isExplored[rightSensorPosition[0]][rightSensorPosition[1] - i] = true;
				 * _mazeRef[rightSensorPosition[0]][rightSensorPosition[1] - i]= IS_EMPTY;
				 * weightageRef[rightSensorPosition[0]][rightSensorPosition[1] - i] -= 1; } if
				 * (numOfClearGrids < Sensor.SHORT_RANGE && rightSensorPosition[1] -
				 * numOfClearGrids - 1 >= 0) {
				 * _isExplored[rightSensorPosition[0]][rightSensorPosition[1] - numOfClearGrids
				 * - 1] = true;
				 * 
				 * weightageRef[rightSensorPosition[0]][rightSensorPosition[1] - numOfClearGrids
				 * - 1] += 1;
				 * 
				 * if((weightageRef[rightSensorPosition[0]][rightSensorPosition[1] -
				 * numOfClearGrids - 1] > 0 ))
				 * _mazeRef[rightSensorPosition[0]][rightSensorPosition[1] - numOfClearGrids -
				 * 1] = IS_OBSTACLE; else
				 * _mazeRef[rightSensorPosition[0]][rightSensorPosition[1] - numOfClearGrids -
				 * 1] = IS_EMPTY; }
				 */

			}
			if (!RobotSystem.isRealRun()) {
				numOfClearGrids = _robot.senseRight(rightSensor2Position, ori);
			} else {
				// System.out.println(numOfClearGrids + "BEFORE R2 NUMBER OF GRIDS");
				// System.out.println(msgSensorValues + "BEFORE R2 SENSOR VALUES");
				numOfClearGrids = parseSensorValue(msgSensorValues, SensorPosition.R2);
				// System.out.println(numOfClearGrids + "AFTER R2 NUMBER OF GRIDS");
				// System.out.println(msgSensorValues + "AFTER R2 SENSOR VALUES");
			}
			if (numOfClearGrids != INVALID_SENSOR_VALUE && rightSensor2Position[1] - numOfClearGrids >= 0) {
				for (int i = 2; i <= numOfClearGrids; i++) {
					_isExplored[rightSensor2Position[0]][rightSensor2Position[1] - i] = true;
					_mazeRef[rightSensor2Position[0]][rightSensor2Position[1] - i] = IS_EMPTY;
				}
				if (numOfClearGrids < Sensor.SHORT_RANGE && rightSensor2Position[1] - numOfClearGrids - 1 >= 0) {
					_isExplored[rightSensor2Position[0]][rightSensor2Position[1] - numOfClearGrids - 1] = true;
					_mazeRef[rightSensor2Position[0]][rightSensor2Position[1] - numOfClearGrids - 1] = IS_OBSTACLE;
				}

				/*
				 * for (int i = 2; i <= numOfClearGrids; i++) {
				 * _isExplored[rightSensor2Position[0]][rightSensor2Position[1] - i] = true;
				 * _mazeRef[rightSensor2Position[0]][rightSensor2Position[1] - i]= IS_EMPTY;
				 * weightageRef[rightSensor2Position[0]][rightSensor2Position[1] - i] -= 1; } if
				 * (numOfClearGrids < Sensor.SHORT_RANGE && rightSensor2Position[1] -
				 * numOfClearGrids - 1 >= 0) {
				 * _isExplored[rightSensor2Position[0]][rightSensor2Position[1] -
				 * numOfClearGrids - 1] = true;
				 * 
				 * weightageRef[rightSensor2Position[0]][rightSensor2Position[1] -
				 * numOfClearGrids - 1] += 1;
				 * 
				 * if((weightageRef[rightSensor2Position[0]][rightSensor2Position[1] -
				 * numOfClearGrids - 1] > 0 ))
				 * _mazeRef[rightSensor2Position[0]][rightSensor2Position[1] - numOfClearGrids -
				 * 1] = IS_OBSTACLE; else
				 * _mazeRef[rightSensor2Position[0]][rightSensor2Position[1] - numOfClearGrids -
				 * 1] = IS_EMPTY; }
				 */
			}
			break;

		case WEST:
			System.out.println("WEST ");
			frontSensorPosition[0] = robotPosition[0];
			frontSensorPosition[1] = robotPosition[1];
			frontleftSensorPosition[0] = robotPosition[0];
			frontleftSensorPosition[1] = robotPosition[1] - 1;
			frontrightSensorPosition[0] = robotPosition[0];
			frontrightSensorPosition[1] = robotPosition[1] + 1;
			leftSensorPosition[0] = robotPosition[0] - 1;
			leftSensorPosition[1] = robotPosition[1];
			rightSensorPosition[0] = robotPosition[0] - 1;
			rightSensorPosition[1] = robotPosition[1];
			rightSensor2Position[0] = robotPosition[0] + 1;
			rightSensor2Position[1] = robotPosition[1];
			if (!RobotSystem.isRealRun()) {
				numOfClearGrids = _robot.senseFront(frontSensorPosition, ori);
			} else {
				numOfClearGrids = parseSensorValue(msgSensorValues, SensorPosition.CF);
			}

			if (numOfClearGrids != INVALID_SENSOR_VALUE && frontSensorPosition[0] - numOfClearGrids >= 0) {
				for (int i = 2; i <= numOfClearGrids; i++) {
					_isExplored[frontSensorPosition[0] - i][frontSensorPosition[1]] = true;
					_mazeRef[frontSensorPosition[0] - i][frontSensorPosition[1]] = IS_EMPTY;
				}
				if (numOfClearGrids < Sensor.SHORT_RANGE && frontSensorPosition[0] - numOfClearGrids - 1 >= 0) {
					_isExplored[frontSensorPosition[0] - numOfClearGrids - 1][frontSensorPosition[1]] = true;
					_mazeRef[frontSensorPosition[0] - numOfClearGrids - 1][frontSensorPosition[1]] = IS_OBSTACLE;
				}

				/*
				 * for (int i = 2; i <= numOfClearGrids; i++) {
				 * _isExplored[frontSensorPosition[0] - i][frontSensorPosition[1]] = true;
				 * _mazeRef[frontSensorPosition[0] - i][frontSensorPosition[1]]= IS_EMPTY;
				 * weightageRef[frontSensorPosition[0] - i][frontSensorPosition[1]]-= 1; } if
				 * (numOfClearGrids < Sensor.SHORT_RANGE && frontSensorPosition[0] -
				 * numOfClearGrids - 1 >= 0) { _isExplored[frontSensorPosition[0] -
				 * numOfClearGrids - 1][frontSensorPosition[1]] = true;
				 * weightageRef[frontSensorPosition[0] - numOfClearGrids -
				 * 1][frontSensorPosition[1]]+= 1;
				 * 
				 * if((weightageRef[frontSensorPosition[0] - numOfClearGrids -
				 * 1][frontSensorPosition[1]] > 0)) _mazeRef[frontSensorPosition[0] -
				 * numOfClearGrids - 1][frontSensorPosition[1]]= IS_OBSTACLE; else
				 * _mazeRef[frontSensorPosition[0] - numOfClearGrids -
				 * 1][frontSensorPosition[1]]= IS_EMPTY; }
				 */

			}
			if (!RobotSystem.isRealRun()) {
				numOfClearGrids = _robot.senseSideFront(frontleftSensorPosition, ori);
			} else {
				numOfClearGrids = parseSensorValue(msgSensorValues, SensorPosition.LF);
			}
			if (numOfClearGrids != INVALID_SENSOR_VALUE && frontleftSensorPosition[0] - numOfClearGrids >= 0) {

				for (int i = 2; i <= numOfClearGrids; i++) {
					_isExplored[frontleftSensorPosition[0] - i][frontleftSensorPosition[1]] = true;
					_mazeRef[frontleftSensorPosition[0] - i][frontleftSensorPosition[1]] = IS_EMPTY;
				}
				if (numOfClearGrids < Sensor.SHORT_RANGE && frontleftSensorPosition[0] - numOfClearGrids - 1 >= 0) {
					_isExplored[frontleftSensorPosition[0] - numOfClearGrids - 1][frontleftSensorPosition[1]] = true;
					_mazeRef[frontleftSensorPosition[0] - numOfClearGrids
							- 1][frontleftSensorPosition[1]] = IS_OBSTACLE;
				}

				/*
				 * for (int i = 2; i <= numOfClearGrids; i++) {
				 * _isExplored[frontleftSensorPosition[0] - i][frontleftSensorPosition[1]] =
				 * true; _mazeRef[frontleftSensorPosition[0] - i][frontleftSensorPosition[1]]=
				 * IS_EMPTY; weightageRef[frontleftSensorPosition[0] -
				 * i][frontleftSensorPosition[1]] -= 1; } if (numOfClearGrids <
				 * Sensor.SHORT_RANGE && frontleftSensorPosition[0] - numOfClearGrids - 1 >= 0)
				 * { _isExplored[frontleftSensorPosition[0] - numOfClearGrids -
				 * 1][frontleftSensorPosition[1]] = true;
				 * weightageRef[frontleftSensorPosition[0] - numOfClearGrids -
				 * 1][frontleftSensorPosition[1]] += 1;
				 * 
				 * if((weightageRef[frontleftSensorPosition[0] - numOfClearGrids -
				 * 1][frontleftSensorPosition[1]] > 0)) _mazeRef[frontleftSensorPosition[0] -
				 * numOfClearGrids - 1][frontleftSensorPosition[1]]= IS_OBSTACLE; else
				 * _mazeRef[frontleftSensorPosition[0] - numOfClearGrids -
				 * 1][frontleftSensorPosition[1]]= IS_EMPTY; }
				 */

			}
			if (!RobotSystem.isRealRun()) {
				numOfClearGrids = _robot.senseSideFront(frontrightSensorPosition, ori);
			} else {
				numOfClearGrids = parseSensorValue(msgSensorValues, SensorPosition.RF);
			}
			if (numOfClearGrids != INVALID_SENSOR_VALUE && frontrightSensorPosition[0] - numOfClearGrids >= 0) {

				for (int i = 2; i <= numOfClearGrids; i++) {
					_isExplored[frontrightSensorPosition[0] - i][frontrightSensorPosition[1]] = true;
					_mazeRef[frontrightSensorPosition[0] - i][frontrightSensorPosition[1]] = IS_EMPTY;
				}
				if (numOfClearGrids < Sensor.SHORT_RANGE && frontrightSensorPosition[0] - numOfClearGrids - 1 >= 0) {
					_isExplored[frontrightSensorPosition[0] - numOfClearGrids - 1][frontrightSensorPosition[1]] = true;
					_mazeRef[frontrightSensorPosition[0] - numOfClearGrids
							- 1][frontrightSensorPosition[1]] = IS_OBSTACLE;
				}

				/*
				 * for (int i = 2; i <= numOfClearGrids; i++) {
				 * _isExplored[frontrightSensorPosition[0] - i][frontrightSensorPosition[1]] =
				 * true; _mazeRef[frontrightSensorPosition[0] - i][frontrightSensorPosition[1]]=
				 * IS_EMPTY; weightageRef[frontrightSensorPosition[0] -
				 * i][frontrightSensorPosition[1]] -= 1; } if (numOfClearGrids <
				 * Sensor.SHORT_RANGE && frontrightSensorPosition[0] - numOfClearGrids - 1 >= 0)
				 * { _isExplored[frontrightSensorPosition[0] - numOfClearGrids -
				 * 1][frontrightSensorPosition[1]] = true;
				 * 
				 * weightageRef[frontrightSensorPosition[0] - numOfClearGrids -
				 * 1][frontrightSensorPosition[1]] += 1;
				 * 
				 * if((weightageRef[frontrightSensorPosition[0] - numOfClearGrids -
				 * 1][frontrightSensorPosition[1]] > 0)) _mazeRef[frontrightSensorPosition[0] -
				 * numOfClearGrids - 1][frontrightSensorPosition[1]]= IS_OBSTACLE; else
				 * _mazeRef[frontrightSensorPosition[0] - numOfClearGrids -
				 * 1][frontrightSensorPosition[1]]= IS_EMPTY;
				 * 
				 * }
				 */
			}
			if (!RobotSystem.isRealRun()) {
				numOfClearGrids = _robot.senseLeft(leftSensorPosition, ori);
			} else {
				numOfClearGrids = parseSensorValue(msgSensorValues, SensorPosition.L);
			}
			if (numOfClearGrids != INVALID_SENSOR_VALUE && leftSensorPosition[1] - numOfClearGrids >= 0) {
				for (int i = 2; i <= numOfClearGrids; i++) {
					_isExplored[leftSensorPosition[0]][leftSensorPosition[1] - i] = true;
					_mazeRef[leftSensorPosition[0]][leftSensorPosition[1] - i] = IS_EMPTY;
				}
				if (numOfClearGrids < Sensor.LONG_RANGE && leftSensorPosition[1] - numOfClearGrids - 1 >= 0) {
					_isExplored[leftSensorPosition[0]][leftSensorPosition[1] - numOfClearGrids - 1] = true;
					_mazeRef[leftSensorPosition[0]][leftSensorPosition[1] - numOfClearGrids - 1] = IS_OBSTACLE;
				}

				/*
				 * for (int i = 2; i <= numOfClearGrids; i++) {
				 * _isExplored[leftSensorPosition[0]][leftSensorPosition[1] - i] = true;
				 * _mazeRef[leftSensorPosition[0]][leftSensorPosition[1] - i] = IS_EMPTY;
				 * weightageRef[leftSensorPosition[0]][leftSensorPosition[1] - i] -= 1; } if
				 * (numOfClearGrids < Sensor.LONG_RANGE && leftSensorPosition[1] -
				 * numOfClearGrids - 1 >= 0) {
				 * _isExplored[leftSensorPosition[0]][leftSensorPosition[1] - numOfClearGrids -
				 * 1] = true;
				 * 
				 * weightageRef[leftSensorPosition[0]][leftSensorPosition[1] - numOfClearGrids -
				 * 1] += 1;
				 * 
				 * if((weightageRef[leftSensorPosition[0]][leftSensorPosition[1] -
				 * numOfClearGrids - 1] > 0)) {
				 * _mazeRef[leftSensorPosition[0]][leftSensorPosition[1] - numOfClearGrids - 1]=
				 * IS_OBSTACLE; leftGotObstacle = true; leftObsPos[0] = leftSensorPosition[0];
				 * leftObsPos[1] = leftSensorPosition[1] - numOfClearGrids - 1; } else {
				 * _mazeRef[leftSensorPosition[0]][leftSensorPosition[1] - numOfClearGrids - 1]=
				 * IS_EMPTY; leftTurnCounter = 0; } }
				 */
			}
			if (!RobotSystem.isRealRun()) {
				numOfClearGrids = _robot.senseRight(rightSensorPosition, ori);
			} else {
				numOfClearGrids = parseSensorValue(msgSensorValues, SensorPosition.R);
			}

			if (numOfClearGrids != INVALID_SENSOR_VALUE && rightSensorPosition[1] + numOfClearGrids < Arena.MAP_WIDTH) {
				for (int i = 2; i <= numOfClearGrids; i++) {
					_isExplored[rightSensorPosition[0]][rightSensorPosition[1] + i] = true;
					_mazeRef[rightSensorPosition[0]][rightSensorPosition[1] + i] = IS_EMPTY;
				}
				if (numOfClearGrids < Sensor.SHORT_RANGE
						&& rightSensorPosition[1] + numOfClearGrids + 1 < Arena.MAP_LENGTH) {
					_isExplored[rightSensorPosition[0]][rightSensorPosition[1] + numOfClearGrids + 1] = true;
					_mazeRef[rightSensorPosition[0]][rightSensorPosition[1] + numOfClearGrids + 1] = IS_OBSTACLE;
				}

				/*
				 * for (int i = 2; i <= numOfClearGrids; i++) {
				 * _isExplored[rightSensorPosition[0]][rightSensorPosition[1] + i] = true;
				 * _mazeRef[rightSensorPosition[0]][rightSensorPosition[1] + i]= IS_EMPTY;
				 * weightageRef[rightSensorPosition[0]][rightSensorPosition[1] + i] -= 1; } if
				 * (numOfClearGrids < Sensor.SHORT_RANGE && rightSensorPosition[1] +
				 * numOfClearGrids + 1 < Arena.MAP_WIDTH) {
				 * 
				 * _isExplored[rightSensorPosition[0]][rightSensorPosition[1] + numOfClearGrids
				 * + 1] = true;
				 * 
				 * weightageRef[rightSensorPosition[0]][rightSensorPosition[1] + numOfClearGrids
				 * + 1] += 1;
				 * 
				 * if((weightageRef[rightSensorPosition[0]][rightSensorPosition[1] +
				 * numOfClearGrids + 1] > 0))
				 * _mazeRef[rightSensorPosition[0]][rightSensorPosition[1] + numOfClearGrids +
				 * 1]= IS_OBSTACLE; else _mazeRef[rightSensorPosition[0]][rightSensorPosition[1]
				 * + numOfClearGrids + 1]= IS_EMPTY; }
				 */
			}
			if (!RobotSystem.isRealRun()) {
				numOfClearGrids = _robot.senseRight(rightSensor2Position, ori);
			} else {
				numOfClearGrids = parseSensorValue(msgSensorValues, SensorPosition.R2);
			}

			if (numOfClearGrids != INVALID_SENSOR_VALUE
					&& rightSensor2Position[1] + numOfClearGrids < Arena.MAP_WIDTH) {

				for (int i = 2; i <= numOfClearGrids; i++) {
					_isExplored[rightSensor2Position[0]][rightSensor2Position[1] + i] = true;
					_mazeRef[rightSensor2Position[0]][rightSensor2Position[1] + i] = IS_EMPTY;
				}
				if (numOfClearGrids < Sensor.SHORT_RANGE
						&& rightSensor2Position[1] + numOfClearGrids + 1 < Arena.MAP_LENGTH) {
					_isExplored[rightSensor2Position[0]][rightSensor2Position[1] + numOfClearGrids + 1] = true;
					_mazeRef[rightSensor2Position[0]][rightSensor2Position[1] + numOfClearGrids + 1] = IS_OBSTACLE;
				}

				/*
				 * for (int i = 2; i <= numOfClearGrids; i++) {
				 * _isExplored[rightSensor2Position[0]][rightSensor2Position[1] + i] = true;
				 * _mazeRef[rightSensor2Position[0]][rightSensor2Position[1] + i]= IS_EMPTY;
				 * weightageRef[rightSensor2Position[0]][rightSensor2Position[1] + i] -= 1; } if
				 * (numOfClearGrids < Sensor.SHORT_RANGE && rightSensor2Position[1] +
				 * numOfClearGrids + 1 < Arena.MAP_WIDTH) {
				 * 
				 * _isExplored[rightSensor2Position[0]][rightSensor2Position[1] +
				 * numOfClearGrids + 1] = true;
				 * 
				 * weightageRef[rightSensor2Position[0]][rightSensor2Position[1] +
				 * numOfClearGrids + 1] += 1;
				 * 
				 * if((weightageRef[rightSensor2Position[0]][rightSensor2Position[1] +
				 * numOfClearGrids + 1] > 0))
				 * _mazeRef[rightSensor2Position[0]][rightSensor2Position[1] + numOfClearGrids +
				 * 1]= IS_OBSTACLE; else
				 * _mazeRef[rightSensor2Position[0]][rightSensor2Position[1] + numOfClearGrids +
				 * 1]= IS_EMPTY; }
				 */
			}

			Controller controller = Controller.getInstance();
			controller.updateMazeColor();

			// if(leftObstacleSelected) {
			// sendObsPosLeft(robotPosition, ori, leftObsPos[0], leftObsPos[1],
			// numOfClearGrids);
			// }

			// System.out.println("Dead end: "+isDeadEnd(controller.getPosition(),
			// controller.getOrientation()));
			/*
			 * if (RobotSystem.isRealRun() && hasCalibration) { // Movement rightCali =
			 * canCalibrateRight(controller.getPosition(), controller.getOrientation()); //
			 * if(rightCali == Movement.LR) { // _robot.calibrateRobotPosition(Movement.Z);
			 * // }
			 * 
			 * Movement move; if (RobotSystem.isRealRun() && hasCalibration) { if
			 * (canCalibrateAhead(msgSensorValues)) { //_robot.calibrateRobotPosition();
			 * move = canCalibrateAside(robotPosition, ori); if (move != null) { if (move ==
			 * Movement.TURN_LEFT) { _robot.turnLeft(); _robot.calibrateRobotPosition();
			 * //_robot.turnRight(); } else if (move == Movement.TURN_RIGHT) {
			 * _robot.turnRight(); //_robot.calibrateRobotPosition(); //_robot.turnLeft(); }
			 * } _robot.resetStepsSinceLastCalibration(); } else { boolean needCalibration =
			 * _robot.getStepsSinceLastCalibration() > CALIBRATION_THRESHOLD; move =
			 * canCalibrateAside(robotPosition, ori);
			 * 
			 * if (needCalibration && move != null) { if (move == Movement.TURN_LEFT) {
			 * //_robot.calibrateRobotPosition(); _robot.turnLeft();
			 * _robot.calibrateRobotPosition(); //_robot.turnRight();
			 * _robot.resetStepsSinceLastCalibration(); } else if (move ==
			 * Movement.TURN_RIGHT) { //_robot.calibrateRobotPosition(); _robot.turnRight();
			 * //_robot.calibrateRobotPosition(); //_robot.turnLeft();
			 * _robot.resetStepsSinceLastCalibration(); } } } } }
			 */
			// }
		}
	}

	/**
	 * set coods for robot in arrayListOfImageRef to capture images from these coods
	 * and orientation check in order of south east north west
	 */
	public void setImageRef() {
		int offsetWall = 0;
		boolean alongWestWall = false;
		boolean alongEastWall = false;
		boolean alongNorthWall = false;
		boolean alongSouthWall = false;
		int offset2 = 0;
		for (int i = 0; i < Arena.MAP_LENGTH; i++) {
			for (int j = 0; j < Arena.MAP_WIDTH; j++) {
				if (_mazeRef[i][j] == IS_OBSTACLE) {
					// check south
					// exit condition based on minimum space for robot
					offset2 = -1;
					if (j + offset2 -2 >= 0 && _mazeRef[i][j + offset2] != IS_OBSTACLE) {
						ImageRef imageRef = new ImageRef();
						offsetWall = 0;
						alongWestWall = false;
						alongEastWall = false;
						// obstacle is along west wall
						if (i == 0) {
							++offsetWall;
							alongWestWall = true;
						} else if (i == Arena.MAP_LENGTH - 1) {
							--offsetWall;
							alongEastWall = true;
						}
						// found empty space south of obstacle, set it as coods to take picture from
						// facing north
						for (int k = 0; k < 3; ++k) {
							if (alongWestWall || alongEastWall)
								k = 3;
							// check west space
							else if (i > 1 && k == 1)
								offsetWall = -1;
							// check east space
							else if (i < Arena.MAP_LENGTH - 2 && k == 2)
								offsetWall = 1;
							if (_mazeRef[i + offsetWall][j + offset2] != IS_OBSTACLE
									&& _mazeRef[i + offsetWall][j + offset2 - 1] != IS_OBSTACLE
									&& _mazeRef[i + offsetWall][j + offset2 - 2] != IS_OBSTACLE
									&& _mazeRef[i + offsetWall - 1][j + offset2] != IS_OBSTACLE
									&& _mazeRef[i + offsetWall - 1][j + offset2 - 1] != IS_OBSTACLE
									&& _mazeRef[i + offsetWall - 1][j + offset2 - 2] != IS_OBSTACLE
									&& _mazeRef[i + offsetWall + 1][j + offset2] != IS_OBSTACLE
									&& _mazeRef[i + offsetWall + 1][j + offset2 - 1] != IS_OBSTACLE
									&& _mazeRef[i + offsetWall + 1][j + offset2 - 2] != IS_OBSTACLE) {

								// check for extra 1 space, required if obstacle is along wall
								if (j + offset2 - 3 >= 0 && _mazeRef[i + offsetWall - 1][j + offset2 - 3] != IS_OBSTACLE
										&& _mazeRef[i + offsetWall][j + offset2 - 3] != IS_OBSTACLE
										&& _mazeRef[i + offsetWall + 1][j + offset2 - 3] != IS_OBSTACLE) {
									imageRef = new ImageRef(i + offsetWall, j + offset2 - 2, Orientation.NORTH,
											i + offsetWall, j + offset2 + 1, Orientation.SOUTH);
									break;
								} else if (alongWestWall == false && alongEastWall == false && k == 0) {
									imageRef = new ImageRef(i, j + offset2 - 1, Orientation.NORTH, i, j + offset2 + 1,
											Orientation.SOUTH);
									break;
								}
							}
						}
						// check if already exists
						if (!arrayListOfImageRefs.contains(imageRef)) {
							ImageRef tempImageRef = new ImageRef();
							if (!imageRef.equalsAbsolute(tempImageRef)) {
								arrayListOfImageRefs.add(imageRef);
								System.out.println("Image Ref: " + imageRef.getX() + "," + imageRef.getY() + ","
										+ imageRef.getOrientation());
								System.out.println("setimageref");
							}
						}
					}
					// check east
					// exit condition based on minimum space for robot
					offset2 = 1;
					if (i + offset2 +2 < Arena.MAP_LENGTH && _mazeRef[i + offset2][j] != IS_OBSTACLE) {
						ImageRef imageRef = new ImageRef();
						offsetWall = 0;
						alongNorthWall = false;
						alongSouthWall = false;
						// obstacle is along south wall
						if (j == 0) {
							++offsetWall;
							alongSouthWall = true;
						} else if (j == Arena.MAP_WIDTH - 1) {
							--offsetWall;
							alongNorthWall = true;
						}
						// found sufficient empty space east of obstacle, set it as coods to take
						// picture from facing south
						for (int k = 0; k < 3; ++k) {
							if (alongNorthWall || alongSouthWall)
								k = 3;
							// check south space
							else if (j > 1 && k == 1)
								offsetWall = -1;
							// check north space
							else if (j < Arena.MAP_WIDTH - 2 && k == 2)
								offsetWall = 1;
							if (_mazeRef[i + offset2][j + offsetWall] != IS_OBSTACLE
									&& _mazeRef[i + offset2 + 1][j + offsetWall] != IS_OBSTACLE
									&& _mazeRef[i + offset2 + 2][j + offsetWall] != IS_OBSTACLE
									&& _mazeRef[i + offset2][j + offsetWall - 1] != IS_OBSTACLE
									&& _mazeRef[i + offset2 + 1][j + offsetWall - 1] != IS_OBSTACLE
									&& _mazeRef[i + offset2 + 2][j + offsetWall - 1] != IS_OBSTACLE
									&& _mazeRef[i + offset2][j + offsetWall + 1] != IS_OBSTACLE
									&& _mazeRef[i + offset2 + 1][j + offsetWall + 1] != IS_OBSTACLE
									&& _mazeRef[i + offset2 + 2][j + offsetWall + 1] != IS_OBSTACLE) {

								// check for extra 1 space
								if (i + 3 + offset2 < Arena.MAP_LENGTH
										&& _mazeRef[i + offset2 + 3][j + offsetWall - 1] != IS_OBSTACLE
										&& _mazeRef[i + offset2 + 3][j + offsetWall] != IS_OBSTACLE
										&& _mazeRef[i + offset2 + 3][j + offsetWall + 1] != IS_OBSTACLE) {
									imageRef = new ImageRef(i + offset2 + 2, j + offsetWall, Orientation.WEST,
											i + offset2 - 1, j + offsetWall, Orientation.EAST);
									break;
								} else if (alongSouthWall == false && alongNorthWall == false && k == 0) {
									imageRef = new ImageRef(i + offset2 + 1, j, Orientation.WEST, i + offset2 - 1, j,
											Orientation.EAST);
									break;
								}
							}
						}
						// check if already exists
						if (!arrayListOfImageRefs.contains(imageRef)) {
							ImageRef tempImageRef = new ImageRef();
							if (!imageRef.equalsAbsolute(tempImageRef)) {
								arrayListOfImageRefs.add(imageRef);
								System.out.println("Image Ref: " + imageRef.getX() + "," + imageRef.getY() + ","
										+ imageRef.getOrientation());
								System.out.println("setimageref2");
							}
						}
					}
					// check north
					// exit condition based on minimum space for robot
					offset2 = 1;
					if (j + offset2 +2 < Arena.MAP_WIDTH && _mazeRef[i][j + offset2] != IS_OBSTACLE) {
						ImageRef imageRef = new ImageRef();
						offsetWall = 0;
						alongWestWall = false;
						alongEastWall = false;
						// obstacle is along west wall
						if (i == 0) {
							++offsetWall;
							alongWestWall = true;
						} else if (i == Arena.MAP_LENGTH - 1) {
							--offsetWall;
							alongEastWall = true;
						}
						// found empty space north of obstacle, set it as coods to take picture from
						// facing south
						for (int k = 0; k < 3; ++k) {
							if (alongWestWall || alongEastWall)
								k = 3;
							// check west space
							else if (i > 1 && k == 1)
								offsetWall = -1;
							// check south space
							else if (i < Arena.MAP_LENGTH - 2 && k == 2)
								offsetWall = 1;
							if (_mazeRef[i + offsetWall][j + offset2] != IS_OBSTACLE
									&& _mazeRef[i + offsetWall][j + offset2 + 1] != IS_OBSTACLE
									&& _mazeRef[i + offsetWall][j + offset2 + 2] != IS_OBSTACLE
									&& _mazeRef[i + offsetWall - 1][j + offset2] != IS_OBSTACLE
									&& _mazeRef[i + offsetWall - 1][j + offset2 + 1] != IS_OBSTACLE
									&& _mazeRef[i + offsetWall - 1][j + offset2 + 2] != IS_OBSTACLE
									&& _mazeRef[i + offsetWall + 1][j + offset2] != IS_OBSTACLE
									&& _mazeRef[i + offsetWall + 1][j + offset2 + 1] != IS_OBSTACLE
									&& _mazeRef[i + offsetWall + 1][j + offset2 + 2] != IS_OBSTACLE) {
								// check for extra 1 space
								if (j + 3 + offset2 < Arena.MAP_WIDTH
										&& _mazeRef[i + offsetWall - 1][j + offset2 + 3] != IS_OBSTACLE
										&& _mazeRef[i + offsetWall][j + offset2 + 3] != IS_OBSTACLE
										&& _mazeRef[i + offsetWall + 1][j + offset2 + 3] != IS_OBSTACLE) {
									imageRef = new ImageRef(i + offsetWall, j + offset2 + 2, Orientation.SOUTH,
											i + offsetWall, j + offset2 - 1, Orientation.NORTH);
									break;
								} else if (alongWestWall == false && alongEastWall == false && k == 0) {
									imageRef = new ImageRef(i, j + offset2 + 1, Orientation.SOUTH, i, j + offset2 - 1,
											Orientation.NORTH);
									break;
								}
							}
						}
						// check if already exists
						if (!arrayListOfImageRefs.contains(imageRef)) {
							ImageRef tempImageRef = new ImageRef();
							if (!imageRef.equalsAbsolute(tempImageRef)) {
								arrayListOfImageRefs.add(imageRef);
								System.out.println("Image Ref: " + imageRef.getX() + "," + imageRef.getY() + ","
										+ imageRef.getOrientation());
								System.out.println("setimageref3");
							}
						}
					}
					// check west
					// exit condition based on minimum space for robot
					System.out.println("after setimageref3");
					offset2 = -1;
					if (i + offset2 - 2 >= 0 && _mazeRef[i + offset2][j] != IS_OBSTACLE) {
						ImageRef imageRef = new ImageRef();
						offsetWall = 0;
						alongNorthWall = false;
						alongSouthWall = false;
						// obstacle is along south wall
						if (j == 0) {
							++offsetWall;
							alongSouthWall = true;
						} else if (j == Arena.MAP_WIDTH - 1) {
							--offsetWall;
							alongNorthWall = true;
						}
						// found sufficient empty space east of obstacle, set it as coods to take
						// picture from facing north
						for (int k = 0; k < 3; ++k) {
							if (alongNorthWall || alongSouthWall)
								k = 3;
							// check south space
							else if (j > 1 && k == 1)
								offsetWall = -1;
							// check north space
							else if (j < Arena.MAP_WIDTH - 2 && k == 2)
								offsetWall = 1;
							if (_mazeRef[i + offset2][j + offsetWall] != IS_OBSTACLE
									&& _mazeRef[i + offset2 - 1][j + offsetWall] != IS_OBSTACLE
									&& _mazeRef[i + offset2 - 2][j + offsetWall] != IS_OBSTACLE
									&& _mazeRef[i + offset2][j + offsetWall - 1] != IS_OBSTACLE
									&& _mazeRef[i + offset2 - 1][j + offsetWall - 1] != IS_OBSTACLE
									&& _mazeRef[i + offset2 - 2][j + offsetWall - 1] != IS_OBSTACLE
									&& _mazeRef[i + offset2][j + offsetWall + 1] != IS_OBSTACLE
									&& _mazeRef[i + offset2 - 1][j + offsetWall + 1] != IS_OBSTACLE
									&& _mazeRef[i + offset2 - 2][j + offsetWall + 1] != IS_OBSTACLE) {
								// check for extra 1 space
								if (i - 3 + offset2 >= 0 && _mazeRef[i + offset2 - 3][j + offsetWall - 1] != IS_OBSTACLE
										&& _mazeRef[i + offset2 - 3][j + offsetWall] != IS_OBSTACLE
										&& _mazeRef[i + offset2 - 3][j + offsetWall + 1] != IS_OBSTACLE) {
									imageRef = new ImageRef(i + offset2 - 2, j + offsetWall, Orientation.EAST,
											i + offset2 + 1, j + offsetWall, Orientation.WEST);
									break;
								} else if (alongSouthWall == false && alongNorthWall == false && k == 0) {
									imageRef = new ImageRef(i + offset2 - 1, j, Orientation.EAST, i + offset2 + 1, j,
											Orientation.WEST);
									break;
								}
							}
						}
						System.out.println("beng is correct");
						// check if already exists
						if (!arrayListOfImageRefs.contains(imageRef)) {
							ImageRef tempImageRef = new ImageRef();
							if (!imageRef.equalsAbsolute(tempImageRef)) {
								arrayListOfImageRefs.add(imageRef);
								System.out.println("Image Ref: " + imageRef.getX() + "," + imageRef.getY() + ","
										+ imageRef.getOrientation());
								System.out.println("setimageref4");
							}
						}
					}
				}
			}
		}
	}
	
	private void sortImageRef() {
		ArrayList<ImageRef> sortedArrayListOfImageRefs = new ArrayList<ImageRef>();
		AStarPathFinder pathFinder = AStarPathFinder.getInstance();
		Path newPath;
		ImageRef currNode = new ImageRef(1, 1, Orientation.EAST);
		Path fastestPath = null;
		int fastestNodeElement = 0;
		while (!arrayListOfImageRefs.isEmpty()) {
			for (int i = 0; i < arrayListOfImageRefs.size(); ++i) {
				newPath = pathFinder.findFastestPath(currNode.getX(), currNode.getY(), currNode.getOrientation(),
						arrayListOfImageRefs.get(i).getX(), arrayListOfImageRefs.get(i).getY(), _mazeRef);
				if (fastestPath == null) {
					fastestPath = newPath;
					fastestNodeElement = 0;
				} else if (newPath.getNumOfSteps() < fastestPath.getNumOfSteps()) {
					fastestPath = newPath;
					fastestNodeElement = i;
				}
			}
			sortedArrayListOfImageRefs.add(new ImageRef(arrayListOfImageRefs.get(fastestNodeElement)));
			currNode = sortedArrayListOfImageRefs.get(sortedArrayListOfImageRefs.size() - 1);
			System.out.println(arrayListOfImageRefs.size());
			arrayListOfImageRefs.remove(fastestNodeElement);
			fastestPath = null;
		}
		arrayListOfImageRefs = new ArrayList<ImageRef>(sortedArrayListOfImageRefs);
	}

	private boolean isThereAdjacentObstacleEastAndWest(int _x, int _y) {
		// out of bounds guard
		if (_x + 1 >= Arena.MAP_LENGTH || _x - 1 < 0 || _y >= Arena.MAP_WIDTH || _y < 0)
			return false;
		if (_mazeRef[_x + 1][_y] == IS_OBSTACLE && _mazeRef[_x - 1][_y] == IS_OBSTACLE)
			return true;
		return false;
	}

	private boolean isThereAdjacentObstacleEastOrWest(int _x, int _y) {
		// out of bounds guard
		if (_y >= Arena.MAP_WIDTH || _y < 0)
			return false;
		if (_x + 1 < Arena.MAP_LENGTH && _mazeRef[_x + 1][_y] == IS_OBSTACLE)
			return true;
		if (_x - 1 >= 0 && _mazeRef[_x - 1][_y] == IS_OBSTACLE)
			return true;
		return false;
	}

	private boolean isThereAdjacentObstacleNorthAndSouth(int _x, int _y) {
		// out of bounds guard
		if (_x >= Arena.MAP_LENGTH || _x < 0 || _y + 1 >= Arena.MAP_WIDTH || _y - 1 < 0)
			return false;
		if (_mazeRef[_x][_y + 1] == IS_OBSTACLE && _mazeRef[_x][_y - 1] == IS_OBSTACLE)
			return true;
		return false;
	}

	private boolean isThereAdjacentObstacleNorthOrSouth(int _x, int _y) {
		// out of bounds guard
		if (_x >= Arena.MAP_LENGTH || _x < 0)
			return false;
		if (_y + 1 < Arena.MAP_WIDTH && _mazeRef[_x][_y + 1] == IS_OBSTACLE)
			return true;
		if (_y - 1 >= 0 && _mazeRef[_x][_y - 1] == IS_OBSTACLE)
			return true;
		return false;
	}

	/**
	 * remove unncessary imageRefs of groupings of 3
	 */
	private void removeUnnecessaryImageRef3() {
		int i = 0;
		while (!arrayListOfImageRefs.isEmpty()) {
			ImageRef imageRefToCheck = arrayListOfImageRefs.get(i);
			// check and remove groups of 3 horizontal imageRef
			if (imageRefToCheck.distanceFromTarget() == 1
					&& isThereAdjacentObstacleEastAndWest(imageRefToCheck.getTargetX(), imageRefToCheck.getTargetY())) {
				for (int j = 0; j < arrayListOfImageRefs.size(); ++j) {
					ImageRef imageRefToCheckAgainst = arrayListOfImageRefs.get(j);
					if (imageRefToCheck.equalsAbsolute(imageRefToCheckAgainst))
						continue;
					if (imageRefToCheck.getOrientation().equals(imageRefToCheckAgainst.getOrientation())) {
						if (imageRefToCheck.isThereAdjacentImageRef(imageRefToCheckAgainst.getX(),
								imageRefToCheckAgainst.getY(), Orientation.EAST)) {
							arrayListOfImageRefs.remove(j);
							--j;
						} else if (imageRefToCheck.isThereAdjacentImageRef(imageRefToCheckAgainst.getX(),
								imageRefToCheckAgainst.getY(), Orientation.WEST)) {
							arrayListOfImageRefs.remove(j);
							--j;
						}
					}
					// exceed arraylist bounds when removing last element
					if (j >= arrayListOfImageRefs.size())
						break;
				}
			}
			// check and remove groups of 3 vertical imageRef
			if (imageRefToCheck.distanceFromTarget() == 1 && isThereAdjacentObstacleNorthAndSouth(
					imageRefToCheck.getTargetX(), imageRefToCheck.getTargetY())) {
				for (int j = 0; j < arrayListOfImageRefs.size(); ++j) {
					ImageRef imageRefToCheckAgainst = arrayListOfImageRefs.get(j);
					if (imageRefToCheck.equalsAbsolute(imageRefToCheckAgainst))
						continue;
					if (imageRefToCheck.getOrientation().equals(imageRefToCheckAgainst.getOrientation())) {
						if (imageRefToCheck.isThereAdjacentImageRef(imageRefToCheckAgainst.getX(),
								imageRefToCheckAgainst.getY(), Orientation.NORTH)) {
							arrayListOfImageRefs.remove(j);
							--j;
						} else if (imageRefToCheck.isThereAdjacentImageRef(imageRefToCheckAgainst.getX(),
								imageRefToCheckAgainst.getY(), Orientation.SOUTH)) {
							arrayListOfImageRefs.remove(j);
							--j;
						}
					}
					// exceed arraylist bounds when removing last element
					if (j >= arrayListOfImageRefs.size())
						break;
				}
			}
			++i;
			if (i >= arrayListOfImageRefs.size())
				return;
		}
	}

	/**
	 * remove unncessary imageRefs of groupings of 2
	 */
	private void removeUnnecessaryImageRef2() {
		int i = 0;
		while (!arrayListOfImageRefs.isEmpty()) {
			ImageRef imageRefToCheck = arrayListOfImageRefs.get(i);
			// check and remove groups of 2 horizontal imageRef
			if (imageRefToCheck.distanceFromTarget() == 1
					&& isThereAdjacentObstacleEastOrWest(imageRefToCheck.getTargetX(), imageRefToCheck.getTargetY())) {
				for (int j = 0; j < arrayListOfImageRefs.size(); ++j) {
					ImageRef imageRefToCheckAgainst = arrayListOfImageRefs.get(j);
					if (imageRefToCheck.equalsAbsolute(imageRefToCheckAgainst)
							|| imageRefToCheck.getTargetY() != imageRefToCheckAgainst.getTargetY())
						continue;
					if (imageRefToCheck.getOrientation().equals(imageRefToCheckAgainst.getOrientation())) {
						if (imageRefToCheck.isThereAdjacentImageRef(imageRefToCheckAgainst.getX(),
								imageRefToCheckAgainst.getY(), Orientation.EAST)) {
							arrayListOfImageRefs.remove(j);
							--j;
						} else if (imageRefToCheck.isThereAdjacentImageRef(imageRefToCheckAgainst.getX(),
								imageRefToCheckAgainst.getY(), Orientation.WEST)) {
							arrayListOfImageRefs.remove(j);
							--j;
						}
					}
					// exceed arraylist bounds when removing last element
					if (j >= arrayListOfImageRefs.size())
						break;
				}
			}
			// check and remove groups of 2 vertical imageRef
			if (imageRefToCheck.distanceFromTarget() == 1 && isThereAdjacentObstacleNorthOrSouth(
					imageRefToCheck.getTargetX(), imageRefToCheck.getTargetY())) {
				for (int j = 0; j < arrayListOfImageRefs.size(); ++j) {
					ImageRef imageRefToCheckAgainst = arrayListOfImageRefs.get(j);
					if (imageRefToCheck.equalsAbsolute(imageRefToCheckAgainst)
							|| imageRefToCheck.getTargetX() != imageRefToCheckAgainst.getTargetX())
						continue;
					if (imageRefToCheck.getOrientation().equals(imageRefToCheckAgainst.getOrientation())) {
						if (imageRefToCheck.isThereAdjacentImageRef(imageRefToCheckAgainst.getX(),
								imageRefToCheckAgainst.getY(), Orientation.NORTH)) {
							arrayListOfImageRefs.remove(j);
							--j;
						} else if (imageRefToCheck.isThereAdjacentImageRef(imageRefToCheckAgainst.getX(),
								imageRefToCheckAgainst.getY(), Orientation.SOUTH)) {
							arrayListOfImageRefs.remove(j);
							--j;
						}
					}
					// exceed arraylist bounds when removing last element
					if (j >= arrayListOfImageRefs.size())
						break;
				}
			}
			++i;
			if (i >= arrayListOfImageRefs.size())
				return;
		}
	}

	/**
	 * after map exploration, if(startImageRun), explore again to find pictures and
	 * send to RPI
	 */
	public void findImage() {
		System.out.println("*** Start findImage()");
		setImageRef();

//		for (int i = 0; i < arrayListOfImageRefs.size(); ++i)
//		{
//			ImageRef imageRef = arrayListOfImageRefs.get(i);
//			System.out.println("Image Ref: " + imageRef.getX() + "," + imageRef.getY() + "," + imageRef.getOrientation() + "," + imageRef.getTargetX() + "," + imageRef.getTargetY() + "," + imageRef.getTargetOrientation());
//		}
//		for (int i = 0; i < arrayListOfImageRefs.size(); ++i) {
//			ImageRef imageRef = arrayListOfImageRefs.get(i);
//			System.out.println("Image Ref: " + imageRef.getX() + "," + imageRef.getY() + "," + imageRef.getOrientation()
//					+ "," + imageRef.getTargetX() + "," + imageRef.getTargetY() + ","
//					+ imageRef.getTargetOrientation());
//		}
		removeUnnecessaryImageRef3();
//		for (int i = 0; i < arrayListOfImageRefs.size(); ++i) {
//			ImageRef imageRef = arrayListOfImageRefs.get(i);
//			System.out.println("Image Ref: " + imageRef.getX() + "," + imageRef.getY() + "," + imageRef.getOrientation()
//					+ "," + imageRef.getTargetX() + "," + imageRef.getTargetY() + ","
//					+ imageRef.getTargetOrientation());
//		}
		removeUnnecessaryImageRef2();
//		for (int i = 0; i < arrayListOfImageRefs.size(); ++i) {
//			ImageRef imageRef = arrayListOfImageRefs.get(i);
//			System.out.println("Image Ref: " + imageRef.getX() + "," + imageRef.getY() + "," + imageRef.getOrientation()
//					+ "," + imageRef.getTargetX() + "," + imageRef.getTargetY() + ","
//					+ imageRef.getTargetOrientation());
//		}
		{
			int i = 0;
			while (i < arrayListOfImageRefsExploration.size())
			{
				int j = 0;
				while (j < arrayListOfImageRefs.size())
				{
					if (arrayListOfImageRefsExploration.get(i).equals(arrayListOfImageRefs.get(j)))
					{
						arrayListOfImageRefsExploration.remove(i);
						arrayListOfImageRefs.remove(j);
					}
					++j;
				}
				++i;
			}
		}
		Collections.sort(arrayListOfImageRefs);
		sortImageRef();
		System.out.println("how about now?");
		for (int i = 0; i < arrayListOfImageRefs.size(); ++i) {
			ImageRef imageRef = arrayListOfImageRefs.get(i);
			System.out.println("Image Ref: " + imageRef.getX() + "," + imageRef.getY() + "," + imageRef.getOrientation()
					+ "," + imageRef.getTargetX() + "," + imageRef.getTargetY() + ","
					+ imageRef.getTargetOrientation());
			System.out.println("pls see me");
		}


		while (!arrayListOfImageRefs.isEmpty()) {
			System.out.println(arrayListOfImageRefs.get(0).getX() + "," + arrayListOfImageRefs.get(0).getY());
			System.out.println("pls see me 2");
			// VirtualMap virtualMap = VirtualMap.getInstance();
			AStarPathFinder pathFinder = AStarPathFinder.getInstance();
			Path fastestPath;
			Controller controller = Controller.getInstance();
			// virtualMap.updateVirtualMap(_mazeRef);

			int obsX = arrayListOfImageRefs.get(0).getX();
			int obsY = arrayListOfImageRefs.get(0).getY();

			System.out.println("Next Point: " + obsX + "," + obsY);
			System.out.println("Time Reach?: " + controller.hasReachedTimeThreshold());
			if (controller.hasReachedTimeThreshold()) {
				System.out.println("Find Image: TIME OUT!");
				break;
			}
			fastestPath = pathFinder.findFastestPath(_robotPosition[0], _robotPosition[1], _robotOrientation, obsX, obsY, _mazeRef);
			System.out.println("MazeExplorer Ori: " + _robotOrientation);
			System.out.println("Controller Ori: " + controller.getOrientation());
			_robotOrientation = pathFinder.moveRobotAlongFastestPath(fastestPath, _robotOrientation, true, true, true);
			// virtualMap.updateVirtualMap(_mazeRef);
			adjustOrientationDynamic(arrayListOfImageRefs.get(0).getOrientation());
			sendPicToRPI(arrayListOfImageRefs.get(0));
			arrayListOfImageRefs.remove(0);
			//_robot.moveForward();
			//_robot.calibrateRobotPositionViaFront();
			//_robot.moveBack();	

		}
		fastestPathBackToStart();
	}

	private Boolean sendPicToRPI(ImageRef _imageRef) {

		if (_imageRef == null) {
			System.out.println("null error");
			return false;
		}

		String msg = _imageRef.getX() + "," + _imageRef.getY() + "," + _imageRef.getOrientation() + ","
				+ _imageRef.getTargetX() + "," + _imageRef.getTargetY() + "," + _imageRef.getTargetOrientation();

		Controller controller = Controller.getInstance();
		PCClient pcClient = controller.getPCClient();

		System.out.println("Send to RPI: " + msg);
		if (RobotSystem.isRealRun()) {
			try {
				Date startDate = new Date();
				Date endDate = null;
				int numSeconds = 0;
				pcClient.sendMsgToRPI(msg);
				System.out.println("Sent Take Picture Command");
				String feedback = pcClient.readMessage();
				System.out.println("Sent Take Picture Command22222");
				while (!feedback.equals(Message.RPI_DONE)) {
					System.out.println("Sent Take Picture Command");
					feedback = pcClient.readMessage();
					System.out.println("Reading Picture Taking Command");
					endDate = new Date();
					numSeconds = (int) ((endDate.getTime() - startDate.getTime()) / 1000);
					System.out.println("Picture Taking Command Timing: " + numSeconds);
					if (((numSeconds % 2) == 0) && numSeconds < 10) {
						pcClient.sendMsgToRPI(msg);
					} else if (numSeconds >= 10) {
						System.out.println("Response timeout");
						return false;
					}

				}
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
		System.out.println("Picture Command Completed");
		// frontcalibrate call here
		// right calirate here
		return true;
	}

	public Orientation updateRobotOrientation(Movement move) {
		switch (move) {
		case TURN_LEFT:
			if (_robotOrientation == Orientation.NORTH) {
				_robotOrientation = Orientation.WEST;
			} else if (_robotOrientation == Orientation.SOUTH) {
				_robotOrientation = Orientation.EAST;
			} else if (_robotOrientation == Orientation.EAST) {
				_robotOrientation = Orientation.NORTH;
			} else {
				_robotOrientation = Orientation.SOUTH;
			}
			break;
		case TURN_RIGHT:
			if (_robotOrientation == Orientation.NORTH) {
				_robotOrientation = Orientation.EAST;
			} else if (_robotOrientation == Orientation.SOUTH) {
				_robotOrientation = Orientation.WEST;
			} else if (_robotOrientation == Orientation.EAST) {
				_robotOrientation = Orientation.SOUTH;
			} else {
				_robotOrientation = Orientation.NORTH;
			}
			break;
		case MOVE_FORWARD:
			break;
		case TURN_RIGHT_TWICE:

		}
		return _robotOrientation;
	}

	public int[] updateRobotPositionAfterMF(Orientation robotOrientation, int[] curRobotPosition) {
		switch (robotOrientation) {
		case NORTH:
			_robotPosition[0] = curRobotPosition[0];
			_robotPosition[1] = curRobotPosition[1] + 1;
			break;
		case SOUTH:
			_robotPosition[0] = curRobotPosition[0];
			_robotPosition[1] = curRobotPosition[1] - 1;
			break;
		case EAST:
			_robotPosition[0] = curRobotPosition[0] + 1;
			_robotPosition[1] = curRobotPosition[1];
			break;
		case WEST:
			_robotPosition[0] = curRobotPosition[0] - 1;
			_robotPosition[1] = curRobotPosition[1];
		}
		return _robotPosition;
	}

	public void adjustOrientationDynamic(Orientation ori) {
		switch (ori) {
		case NORTH:
			if (_robotOrientation == Orientation.SOUTH) {

				_robot.turnRight();
				_robotOrientation = Orientation.WEST;

				_robot.turnRight();
				_robotOrientation = Orientation.NORTH;

			} else if (_robotOrientation == Orientation.EAST) {

				_robot.turnLeft();
				_robotOrientation = Orientation.NORTH;

			} else if (_robotOrientation == Orientation.WEST) {

				_robot.turnRight();
				_robotOrientation = Orientation.NORTH;

			}
			break;
		case SOUTH:
			if (_robotOrientation == Orientation.NORTH) {

				_robot.turnRight();
				_robotOrientation = Orientation.EAST;

				_robot.turnRight();
				_robotOrientation = Orientation.SOUTH;

			} else if (_robotOrientation == Orientation.EAST) {

				_robot.turnRight();
				_robotOrientation = Orientation.SOUTH;

			} else if (_robotOrientation == Orientation.WEST) {
				_robot.turnLeft();
				_robotOrientation = Orientation.SOUTH;
			}
			break;
		case EAST:
			if (_robotOrientation == Orientation.NORTH) {
				_robot.turnRight();
				_robotOrientation = Orientation.EAST;

			} else if (_robotOrientation == Orientation.SOUTH) {

				_robot.turnLeft();
				_robotOrientation = Orientation.EAST;

			} else if (_robotOrientation == Orientation.WEST) {

				_robot.turnRight();
				_robotOrientation = Orientation.NORTH;

				_robot.turnRight();
				_robotOrientation = Orientation.EAST;
			}
			break;
		case WEST:
			if (_robotOrientation == Orientation.NORTH) {
				_robot.turnLeft();
				_robotOrientation = Orientation.WEST;
			} else if (_robotOrientation == Orientation.SOUTH) {

				_robot.turnRight();
				_robotOrientation = Orientation.WEST;
			} else if (_robotOrientation == Orientation.EAST) {

				_robot.turnRight();
				_robotOrientation = Orientation.SOUTH;

				_robot.turnRight();
				_robotOrientation = Orientation.WEST;

			}
		}
	}

	public void adjustOrientationDynamicWithExplore(Orientation ori) {
		switch (ori) {
		case NORTH:
			if (_robotOrientation == Orientation.SOUTH) {

				_robot.turnRight();
				_robotOrientation = Orientation.WEST;
				setIsExplored(_robotPosition, _robotOrientation, true);
				_robot.turnRight();
				_robotOrientation = Orientation.NORTH;
				setIsExplored(_robotPosition, _robotOrientation, true);

			} else if (_robotOrientation == Orientation.EAST) {

				_robot.turnLeft();
				_robotOrientation = Orientation.NORTH;
				setIsExplored(_robotPosition, _robotOrientation, true);

			} else if (_robotOrientation == Orientation.WEST) {

				_robot.turnRight();
				_robotOrientation = Orientation.NORTH;
				setIsExplored(_robotPosition, _robotOrientation, true);
			}
			break;
		case SOUTH:
			if (_robotOrientation == Orientation.NORTH) {

				_robot.turnRight();
				_robotOrientation = Orientation.EAST;
				setIsExplored(_robotPosition, _robotOrientation, true);
				_robot.turnRight();
				_robotOrientation = Orientation.SOUTH;
				
				setIsExplored(_robotPosition, _robotOrientation, true);

			} else if (_robotOrientation == Orientation.EAST) {

				_robot.turnRight();
				_robotOrientation = Orientation.SOUTH;
				setIsExplored(_robotPosition, _robotOrientation, true);

			} else if (_robotOrientation == Orientation.WEST) {
				_robot.turnLeft();
				_robotOrientation = Orientation.SOUTH;
				setIsExplored(_robotPosition, _robotOrientation, true);
			}
			break;
		case EAST:
			if (_robotOrientation == Orientation.NORTH) {
				_robot.turnRight();
				_robotOrientation = Orientation.EAST;
				setIsExplored(_robotPosition, _robotOrientation, true);
			} else if (_robotOrientation == Orientation.SOUTH) {

				_robot.turnLeft();
				_robotOrientation = Orientation.EAST;
				setIsExplored(_robotPosition, _robotOrientation, true);

			} else if (_robotOrientation == Orientation.WEST) {

				_robot.turnRight();
				_robotOrientation = Orientation.NORTH;
				setIsExplored(_robotPosition, _robotOrientation, true);
				_robot.turnRight();
				_robotOrientation = Orientation.EAST;
				setIsExplored(_robotPosition, _robotOrientation, true);
			}
			break;
		case WEST:
			if (_robotOrientation == Orientation.NORTH) {
				_robot.turnLeft();
				_robotOrientation = Orientation.WEST;
				setIsExplored(_robotPosition, _robotOrientation, true);
			} else if (_robotOrientation == Orientation.SOUTH) {

				_robot.turnRight();
				_robotOrientation = Orientation.WEST;
				setIsExplored(_robotPosition, _robotOrientation, true);
			} else if (_robotOrientation == Orientation.EAST) {

				_robot.turnRight();
				_robotOrientation = Orientation.SOUTH;
				setIsExplored(_robotPosition, _robotOrientation, true);
				_robot.turnRight();
				_robotOrientation = Orientation.WEST;
				setIsExplored(_robotPosition, _robotOrientation, true);
			}
		}
	}
	
	public void adjustOrientationTo(Orientation ori) {
		System.out.println("bestori:" + ori);
		_robotOrientation = Orientation.EAST;
		System.out.println("robotori:" + _robotOrientation);
		switch (ori) {
		case NORTH:
			if (_robotOrientation == Orientation.SOUTH) {

				_robot.turnRight();
				_robotOrientation = Orientation.WEST;
				setIsExplored(_robotPosition, _robotOrientation, true);

				_robot.turnRight();
				_robotOrientation = Orientation.NORTH;
				System.out.println("you should NOT see me" + _robotOrientation + "north");
				setIsExplored(_robotPosition, _robotOrientation, true);

			} else if (_robotOrientation == Orientation.EAST) {
				System.out.println("you should see me 1");
				_robot.turnLeft();
				System.out.println("you should see me");
				_robotOrientation = Orientation.NORTH;
				setIsExplored(_robotPosition, _robotOrientation, true);

			} else if (_robotOrientation == Orientation.WEST) {

				_robot.turnRight();
				_robotOrientation = Orientation.NORTH;
				System.out.println("you should NOT see me" + _robotOrientation + "north");
				setIsExplored(_robotPosition, _robotOrientation, true);

			}
			break;
		case SOUTH:
			if (_robotOrientation == Orientation.NORTH) {

				_robot.turnRight();
				_robotOrientation = Orientation.EAST;
				System.out.println("you should NOT see me" + _robotOrientation + "south");
				setIsExplored(_robotPosition, _robotOrientation, true);

				_robot.turnRight();
				_robotOrientation = Orientation.SOUTH;
				System.out.println("you should NOT see me" + _robotOrientation + "south");
				setIsExplored(_robotPosition, _robotOrientation, true);

			} else if (_robotOrientation == Orientation.EAST) {

				_robot.turnRight();
				_robotOrientation = Orientation.SOUTH;
				System.out.println("you should NOT see me" + _robotOrientation + "south");
				setIsExplored(_robotPosition, _robotOrientation, true);

			} else if (_robotOrientation == Orientation.WEST) {
				_robot.turnLeft();
				_robotOrientation = Orientation.SOUTH;
				System.out.println("you should NOT see me" + _robotOrientation + "south");
				setIsExplored(_robotPosition, _robotOrientation, true);
			}
			break;
		case EAST:
			if (_robotOrientation == Orientation.NORTH) {
				_robot.turnRight();
				_robotOrientation = Orientation.EAST;
				System.out.println("you should NOT see me" + _robotOrientation + "east");
				setIsExplored(_robotPosition, _robotOrientation, true);

			} else if (_robotOrientation == Orientation.SOUTH) {

				_robot.turnLeft();
				_robotOrientation = Orientation.EAST;
				System.out.println("you should NOT see me" + _robotOrientation + "east");
				setIsExplored(_robotPosition, _robotOrientation, true);

			} else if (_robotOrientation == Orientation.WEST) {

				_robot.turnRight();
				_robotOrientation = Orientation.NORTH;
				System.out.println("you should NOT see me" + _robotOrientation + "east");
				setIsExplored(_robotPosition, _robotOrientation, true);

				_robot.turnRight();
				_robotOrientation = Orientation.EAST;
				System.out.println("you should NOT see me" + _robotOrientation + "east");
				setIsExplored(_robotPosition, _robotOrientation, true);
			}
			break;
		case WEST:
			if (_robotOrientation == Orientation.NORTH) {
				_robot.turnLeft();
				_robotOrientation = Orientation.WEST;
				System.out.println("you should NOT see me" + _robotOrientation + "west");
				setIsExplored(_robotPosition, _robotOrientation, true);
			} else if (_robotOrientation == Orientation.SOUTH) {

				_robot.turnRight();
				_robotOrientation = Orientation.WEST;
				System.out.println("you should NOT see me" + _robotOrientation + "west");
				setIsExplored(_robotPosition, _robotOrientation, true);
			} else if (_robotOrientation == Orientation.EAST) {

				_robot.turnRight();
				_robotOrientation = Orientation.SOUTH;
				System.out.println("you should NOT see me" + _robotOrientation + "west");
				setIsExplored(_robotPosition, _robotOrientation, true);

				_robot.turnRight();
				_robotOrientation = Orientation.WEST;
				System.out.println("you should NOT see me" + _robotOrientation + "west");
				setIsExplored(_robotPosition, _robotOrientation, true);

			}
		}

	}

	public void init(int[] robotPosition, Orientation robotOrientation) {
		_robot = Robot.getInstance();
		_robotPosition = new int[2];
		_robotPosition[0] = robotPosition[0];
		_robotPosition[1] = robotPosition[1];
		_robotOrientation = robotOrientation;
		_hasExploredTillGoal = false;
		_pathFinder = AStarPathFinder.getInstance();
		lastCalibrate[0] = 2;
		lastCalibrate[1] = 2;
		_isExplored = new Boolean[Arena.MAP_LENGTH][Arena.MAP_WIDTH];
		_mazeRef = new int[Arena.MAP_LENGTH][Arena.MAP_WIDTH];
		rightWallRef = new int[Arena.MAP_LENGTH][Arena.MAP_WIDTH];
		imageRef = new int[Arena.MAP_LENGTH][Arena.MAP_WIDTH];
		weightageRef = new int[Arena.MAP_LENGTH][Arena.MAP_WIDTH];

		for (int i = 0; i < Arena.MAP_LENGTH; i++) {
			for (int j = 0; j < Arena.MAP_WIDTH; j++) {
				_isExplored[i][j] = false;
			}
		}
		for (int i = 0; i < Arena.MAP_LENGTH; i++) {
			for (int j = 0; j < Arena.MAP_WIDTH; j++) {
				_mazeRef[i][j] = UNEXPLORED;
				rightWallRef[i][j] = UNEXPLORED;
				imageRef[i][j] = UNEXPLORED;
				weightageRef[i][j] = 0;
			}
		}
	}

	private boolean ExploreNextRound(int[] currentRobotPosition, HashMap<Integer, int[]> positionHashMap,
			HashMap<Integer, Integer> positionHashMap2) {

		VirtualMap virtualMap = VirtualMap.getInstance();
		AStarPathFinder pathFinder = AStarPathFinder.getInstance();
		Path fastestPath;
		boolean isExhaustedViaFront = true, isExhaustedViaFrontside = true;
		boolean end;
		Controller controller = Controller.getInstance();
		int[] nextRobotPosition = null, checkExist, store = new int[2];
		virtualMap.updateVirtualMap(_mazeRef);

		for (int obsY = 0; obsY < Arena.MAP_WIDTH; obsY++) {
			for (int obsX = 0; obsX < Arena.MAP_LENGTH; obsX++) {
				if (controller.hasReachedTimeThreshold()) {
					return true;
				}
				if (_mazeRef[obsX][obsY] == UNEXPLORED) {
					nextRobotPosition = getNearestRobotPositionTo(obsX, obsY, virtualMap, false);
					if (nextRobotPosition != null) {
						isExhaustedViaFront = false;
					} else {
						 //isExhaustedViaFront = true;
						continue;
					}

					fastestPath = pathFinder.findFastestPath(currentRobotPosition[0], currentRobotPosition[1],
							nextRobotPosition[0], nextRobotPosition[1], _mazeRef);

					_robotOrientation = pathFinder.moveRobotAlongFastestPath(fastestPath, _robotOrientation, true, true,
							false);
					virtualMap.updateVirtualMap(_mazeRef);

					if (_robotPosition[0] > obsX) {
						adjustOrientationDynamicWithExplore(Orientation.WEST);
					} else if (_robotPosition[0] < obsX) {
						adjustOrientationDynamicWithExplore(Orientation.EAST);
					} else if (_robotPosition[1] > obsY) {
						adjustOrientationDynamicWithExplore(Orientation.SOUTH);
					} else if (_robotPosition[1] < obsY) {
						adjustOrientationDynamicWithExplore(Orientation.NORTH);
					}
					isExhaustedViaFront = true;
					currentRobotPosition = nextRobotPosition;

					checkExist = new int[2];
					if (positionHashMap.containsKey(currentRobotPosition[0])) {
						store = positionHashMap.get(currentRobotPosition[0]);
						if (store[0] == currentRobotPosition[1]) {
							store[1]++;
							if (store[1] > 2)
								return true;
							else {
								checkExist[0] = currentRobotPosition[1];
								checkExist[1] = store[1];
								positionHashMap.put(currentRobotPosition[0], checkExist);
							}
						}
					} else {
						checkExist[0] = currentRobotPosition[1];
						checkExist[1] = 1;
						positionHashMap.put(currentRobotPosition[0], checkExist);
					}

				}
			}
		}
		if (isExhaustedViaFront) {
			for (int obsY = 0; obsY < Arena.MAP_WIDTH; obsY++) {
				for (int obsX = 0; obsX < Arena.MAP_LENGTH; obsX++) {
					if (_mazeRef[obsX][obsY] == UNEXPLORED) {

						nextRobotPosition = getNearestRobotPositionTo(obsX, obsY, virtualMap, true);
						if (nextRobotPosition != null) {
							isExhaustedViaFrontside = false;
						} else {
							continue;
						}

						fastestPath = pathFinder.findFastestPath(currentRobotPosition[0], currentRobotPosition[1],
								nextRobotPosition[0], nextRobotPosition[1], _mazeRef);

						_robotOrientation = pathFinder.moveRobotAlongFastestPath(fastestPath, _robotOrientation, true,
								true, false);
						virtualMap.updateVirtualMap(_mazeRef);

						if (_robotPosition[0] > obsX) {
							adjustOrientationDynamicWithExplore(Orientation.WEST);
						} else if (_robotPosition[0] < obsX) {
							adjustOrientationDynamicWithExplore(Orientation.EAST);
						} else if (_robotPosition[1] > obsY) {
							adjustOrientationDynamicWithExplore(Orientation.SOUTH);
						} else if (_robotPosition[1] < obsY) {
							adjustOrientationDynamicWithExplore(Orientation.NORTH);
						}

						currentRobotPosition = nextRobotPosition;
						if (positionHashMap2.containsKey(currentRobotPosition[0])) {
							if (positionHashMap2.get(currentRobotPosition[0]) == currentRobotPosition[1]) {
								return true;
							}
						}
						positionHashMap2.put(currentRobotPosition[0], currentRobotPosition[1]);
					}
				}
			}
		}

		if (isExhaustedViaFront && isExhaustedViaFrontside) {
			System.out.println("ISIT HERE");
			fastestPath = pathFinder.findFastestPath(currentRobotPosition[0], currentRobotPosition[1],
					MazeExplorer.START[0], MazeExplorer.START[1], _mazeRef);
			_robotOrientation = pathFinder.moveRobotAlongFastestPath(fastestPath, _robotOrientation, true, true, false);
			end = true;
		} else
			end = false;

		return end;
	}

	private int[] getNearestRobotPositionTo(int obsX, int obsY, VirtualMap virtualMap, boolean isExhausted) {
		int nearestPosition[] = new int[2];
		boolean[][] cleared = virtualMap.getCleared();
		boolean isClearedAhead;
		for (int radius = 2; radius <= Sensor.LONG_RANGE; radius++) {
			for (int y = 0; y <= obsY + radius; y++) {
				// for (int x = 0; x <= obsX + radius; x++) {
				for (int x = obsX + radius; x >= 0; x--) {
					if (x == obsX - radius || x == obsX + radius || y == obsY - radius || y == obsY + radius) {
						if (x >= 0 && y >= 0 && x < Arena.MAP_LENGTH && y < Arena.MAP_WIDTH) {
							if (cleared[x][y]) {
								if ((Math.abs(obsX + obsY - x - y) == radius) && (x == obsX || y == obsY)) {
									isClearedAhead = true;
									if (x > obsX) {
										for (int i = obsX + 1; i < x; i++) {
											if (_mazeRef[i][y] != IS_EMPTY) {
												isClearedAhead = false;
											}
										}
									} else if (x < obsX) {
										for (int i = x + 1; i < obsX; i++) {
											if (_mazeRef[i][y] != IS_EMPTY) {
												isClearedAhead = false;
											}
										}
									} else if (y > obsY) {
										for (int j = obsY + 1; j < y; j++) {
											if (_mazeRef[x][j] != IS_EMPTY) {
												isClearedAhead = false;
											}
										}
									} else if (y < obsY) {
										for (int j = y + 1; j < obsY; j++) {
											if (_mazeRef[x][j] != IS_EMPTY) {
												isClearedAhead = false;
											}
										}
									}
									if (isClearedAhead) {
										nearestPosition[0] = x;
										nearestPosition[1] = y;
										return nearestPosition;
									}
								}
							}
						}
					}
				}
			}
		}
		if (isExhausted) {
			for (int radius = 2; radius <= Sensor.SHORT_RANGE; radius++) {
				for (int y = obsY - radius; y <= obsY + radius; y++) {
					for (int x = obsX - radius; x <= obsX + radius; x++) {
						if (x == obsX - radius || x == obsX + radius || y == obsY - radius || y == obsY + radius) {
							if (x >= 0 && y >= 0 && x < Arena.MAP_LENGTH && y < Arena.MAP_WIDTH) {
								if (cleared[x][y]) {

									if (Math.abs(x - obsX) + Math.abs(y - obsY) == radius + 1) {

										isClearedAhead = true;
										if (x == obsX + radius && y > obsY) {
											for (int i = obsX + 1; i < x; i++) {
												if (_mazeRef[i][obsY] != IS_EMPTY) {
													isClearedAhead = false;
												}
											}
										} else if (x == obsX + radius && y < obsY) {
											for (int i = obsX + 1; i < x; i++) {
												if (_mazeRef[i][obsY] != IS_EMPTY) {
													isClearedAhead = false;
												}
											}
										} else if (x == obsX - radius && y > obsY) {
											for (int i = x + 1; i < obsX; i++) {
												if (_mazeRef[i][obsY] != IS_EMPTY) {
													isClearedAhead = false;
												}
											}
										} else if (x == obsX - radius && y < obsY) {
											for (int i = x + 1; i < obsX; i++) {
												if (_mazeRef[i][obsY] != IS_EMPTY) {
													isClearedAhead = false;
												}
											}
										} else if (x < obsX && y == obsY + radius) {
											for (int j = obsY + 1; j < y; j++) {
												if (_mazeRef[obsX][j] != IS_EMPTY) {
													isClearedAhead = false;
												}
											}
										} else if (x > obsX && y == obsY + radius) {
											for (int j = obsY + 1; j < y; j++) {
												if (_mazeRef[obsX][j] != IS_EMPTY) {
													isClearedAhead = false;
												}
											}
										} else if (x < obsX && y == obsY - radius) {
											for (int j = y + 1; j < obsY; j++) {
												if (_mazeRef[obsX][j] != IS_EMPTY) {
													isClearedAhead = false;
												}
											}
										} else if (x > obsX && y == obsY - radius) {
											for (int j = y + 1; j < obsY; j++) {
												if (_mazeRef[obsX][j] != IS_EMPTY) {
													isClearedAhead = false;
												}
											}
										}
										if (isClearedAhead) {
											nearestPosition[0] = x;
											nearestPosition[1] = y;

											return nearestPosition;
										}

									}
								}
							}
						}
					}
				}
			}
		}
		return null;
	}

	private boolean checkGhostWall(int[] robotPosition, Orientation ori) {
		int x = robotPosition[0];
		int y = robotPosition[1];

		switch (ori) {
		case NORTH:
			if (_mazeRef[x + 2][y + 1] == IS_EMPTY && (x + 2) != 15)
				return true;
			break;
		case SOUTH:
			if (_mazeRef[x - 2][y - 1] == IS_EMPTY && (x - 2) != -1)
				return true;
			break;
		case EAST:
			if (_mazeRef[x + 1][y - 2] == IS_EMPTY && (y - 2) != -1)
				return true;
			break;
		case WEST:
			if (_mazeRef[x - 1][y + 2] == IS_EMPTY && (y + 2) != 20)
				return true;
			break;
		}
		return false;
	}
	private void exploreAlongWallClean(int[] goalPos) {

		Controller controller = Controller.getInstance();
		int cali_threshold = -1;
		int straight_cali_threshold = 0;
		while (!isGoalPos(_robotPosition, goalPos) && !controller.hasReachedTimeThreshold()) {
			int rightStatus = checkRightSide(_robotPosition, _robotOrientation);
			if (rightStatus != RIGHT_NO_ACCESS) {

				System.out.println("ENTER RIGHT CAN ACCESS" + rightStatus);

				if (rightStatus == RIGHT_UNSURE_ACCESS) {
					System.out.println("ENTER RIGHT CAN ACCESS + UNSURE ACCESS" + rightStatus);
					_robot.turnRight();
					updateRobotOrientation(Movement.TURN_RIGHT);
					setIsExplored(_robotPosition, _robotOrientation, true);
					canTakePicFront(_robotOrientation);
					if (hasAccessibleFront(_robotPosition, _robotOrientation)) {
						System.out.println("ENTER RIGHT CAN ACCESS + UNSURE ACCESS + ACCESSIBLEFRONT" + rightStatus);
						_robot.moveForward();
						_robot.calibrateRobotPosition();
						updateRobotPositionAfterMF(_robotOrientation, _robotPosition);
						setIsExplored(_robotPosition, _robotOrientation, true);
						canTakePicFront(_robotOrientation);
					} else {
						System.out.println("ENTER RIGHT CAN ACCESS + UNSURE ACCESS + NO ACCESSIBLEFRONT" + rightStatus);
						_robot.turnLeft();
						_robot.calibrateRobotPositionViaFront();
						updateRobotOrientation(Movement.TURN_LEFT);
					}
					straight_cali_threshold = 0;
				}

				else { // rightStatus == RIGHT_CANNOT_ACCESS
					System.out.println("ENTER RIGHT CANNOT ACCESS" + rightStatus);
					if (leftWallKiss)
					{
						leftWallKiss = false;
						_robot.turnLeft();
						updateRobotOrientation(Movement.TURN_LEFT);
						_robot.calibrateRobotPositionViaFront();
						_robot.turnRight();
						updateRobotOrientation(Movement.TURN_RIGHT);
					}
					_robot.calibrateRobotPosition();
					_robot.turnRight();

					updateRobotOrientation(Movement.TURN_RIGHT);
					setIsExplored(_robotPosition, _robotOrientation, true);
					canTakePicFront(_robotOrientation);

					if (hasAccessibleFront(_robotPosition, _robotOrientation)) {
						System.out.println("HAF ACCESSIBLE FRONT" + rightStatus);
						_robot.moveForward();
						_robot.calibrateRobotPosition();
						updateRobotPositionAfterMF(_robotOrientation, _robotPosition);
						setIsExplored(_robotPosition, _robotOrientation, true);
						canTakePicFront(_robotOrientation);
					} else {
						System.out.println("NO HAF ACCESSIBLE FRONT" + rightStatus);
						_robot.calibrateRobotPositionViaFront();
						_robot.turnLeft();
						_robot.calibrateRobotPosition();
						straight_cali_threshold = 0;
						updateRobotPositionAfterMF(_robotOrientation, _robotPosition);
						setIsExplored(_robotPosition, _robotOrientation, true);
						canTakePicFront(_robotOrientation);
					}

				}

			}

			else if (hasAccessibleFront(_robotPosition, _robotOrientation)) {	
				if (straight_cali_threshold == 6) {
					//check right sensor both == 1, then do below
					_robot.turnRight();
					_robot.calibrateRobotPositionViaFront();
					_robot.turnLeft();
					_robot.calibrateRobotPosition();
					straight_cali_threshold = 0;
					System.out.println("Calibrating via Front while walking straight paths");
				}
				_robot.moveForward();
				_robot.calibrateRobotPosition();
				updateRobotPositionAfterMF(_robotOrientation, _robotPosition);
				setIsExplored(_robotPosition, _robotOrientation, true);
				canTakePicFront(_robotOrientation);
				straight_cali_threshold++;
			}

			else {
					if (rightStatus == RIGHT_NO_ACCESS && !hasAccessibleFront(_robotPosition, _robotOrientation)) {
						cali_threshold++;
						if (cali_threshold == 0) {
							//check right sensor both == 1, then do below
							_robot.calibrateRobotPositionViaFront();
							_robot.turnRight();
							updateRobotOrientation(Movement.TURN_RIGHT);
							_robot.calibrateRobotPositionViaFront();
							canTakePicFront(_robotOrientation);
							_robot.turnLeft();
							updateRobotOrientation(Movement.TURN_LEFT);
							cali_threshold = -2;
							System.out.println("Calibrating via Front while corner");
						}
						cali_threshold++;
					}
				_robot.turnLeft();
				updateRobotOrientation(Movement.TURN_LEFT);
				if(!hasAccessibleFront(_robotPosition, _robotOrientation))
				{
					leftWallKiss = true;
				}
				setIsExplored(_robotPosition, _robotOrientation, true);
				canTakePicFront(_robotOrientation);
				straight_cali_threshold = 0;
			}
		}

	}
	

	private void exploreAlongWallClean2(int[] goalPos) {

		Controller controller = Controller.getInstance();

		while (!isGoalPos(_robotPosition, goalPos) && !controller.hasReachedTimeThreshold()) {
			int rightStatus = checkRightSide(_robotPosition, _robotOrientation);

			if (rightStatus != RIGHT_NO_ACCESS) {

				System.out.println("ENTER RIGHT CAN ACCESS" + rightStatus);

				if (rightStatus == RIGHT_UNSURE_ACCESS) {
					System.out.println("ENTER RIGHT CAN ACCESS + UNSURE ACCESS" + rightStatus);
					_robot.turnRight();
					updateRobotOrientation(Movement.TURN_RIGHT);
					setIsExplored(_robotPosition, _robotOrientation, true);

					if (hasAccessibleFront(_robotPosition, _robotOrientation)) {
						System.out.println("ENTER RIGHT CAN ACCESS + UNSURE ACCESS + ACCESSIBLEFRONT" + rightStatus);
						_robot.moveForward();
						_robot.calibrateRobotPosition();
						updateRobotPositionAfterMF(_robotOrientation, _robotPosition);
						setIsExplored(_robotPosition, _robotOrientation, true);
					} else {
						System.out.println("ENTER RIGHT CAN ACCESS + UNSURE ACCESS + NO ACCESSIBLEFRONT" + rightStatus);
						_robot.turnLeft();
						_robot.calibrateRobotPositionViaFront();
						updateRobotOrientation(Movement.TURN_LEFT);
					}
				}

				else { // rightStatus == RIGHT_CANNOT_ACCESS
					System.out.println("ENTER RIGHT CANNOT ACCESS" + rightStatus);

					_robot.turnRight();

					updateRobotOrientation(Movement.TURN_RIGHT);
					setIsExplored(_robotPosition, _robotOrientation, true);

					if (hasAccessibleFront(_robotPosition, _robotOrientation)) {
						System.out.println("HAF ACCESSIBLE FRONT" + rightStatus);
						_robot.moveForward();
						_robot.calibrateRobotPosition();
						updateRobotPositionAfterMF(_robotOrientation, _robotPosition);
						setIsExplored(_robotPosition, _robotOrientation, true);
					} else {
						System.out.println("NO HAF ACCESSIBLE FRONT" + rightStatus);
						_robot.turnLeft();
						_robot.calibrateRobotPositionViaFront();
						updateRobotPositionAfterMF(_robotOrientation, _robotPosition);
						setIsExplored(_robotPosition, _robotOrientation, true);
					}

				}

			}

			else if (hasAccessibleFront(_robotPosition, _robotOrientation)) {
				_robot.moveForward();
				_robot.calibrateRobotPosition();
				updateRobotPositionAfterMF(_robotOrientation, _robotPosition);
				setIsExplored(_robotPosition, _robotOrientation, true);
			}

			else {
				_robot.turnLeft();
				updateRobotOrientation(Movement.TURN_LEFT);
				setIsExplored(_robotPosition, _robotOrientation, true);
			}
		}

	}
	private void exploreAlongWall(int[] goalPos) {
		Controller controller = Controller.getInstance();

		while (!isGoalPos(_robotPosition, goalPos) && !controller.hasReachedTimeThreshold()) {
			int rightStatus = checkRightSide(_robotPosition, _robotOrientation);
			updateWall(_robotPosition, _robotOrientation);
			System.out.println("Right Status: " + rightStatus);
			if (rightStatus != RIGHT_NO_ACCESS) {
				System.out.println("Right Have Access");
				if (rightStatus == RIGHT_UNSURE_ACCESS) {
					System.out.println("Right Unsure Access");
					_robot.turnRight();
					updateRobotOrientation(Movement.TURN_RIGHT);
					setIsExplored(_robotPosition, _robotOrientation, true);

					if (hasAccessibleFront(_robotPosition, _robotOrientation)) {
						_robot.moveForward();
						updateRobotPositionAfterMF(_robotOrientation, _robotPosition);
						setIsExplored(_robotPosition, _robotOrientation, true);
					} else {
						_robot.turnLeft();
						updateRobotOrientation(Movement.TURN_LEFT);
					}
				} else { // rightStatus == RIGHT_CAN_ACCESS
					System.out.println("Right Sure Access");

					_robot.turnRight();
					updateRobotOrientation(Movement.TURN_RIGHT);
					setIsExplored(_robotPosition, _robotOrientation, true);

					_robot.moveForward();
					updateRobotPositionAfterMF(_robotOrientation, _robotPosition);
					setIsExplored(_robotPosition, _robotOrientation, true);

					if (checkGhostWall(_robotPosition, _robotOrientation)) {
						AStarPathFinder pathFinder = AStarPathFinder.getInstance();
						Path backPath = pathFinder.findFastestPath(_robotPosition[0], _robotPosition[1],
								lastCalibrate[0], lastCalibrate[1], _mazeRef);
						_robotOrientation = pathFinder.moveRobotAlongFastestPath(backPath, _robotOrientation, false,
								false, false);
						_robotPosition[0] = lastCalibrate[0];
						_robotPosition[1] = lastCalibrate[1];

						_robot.turn180();
						switch (_robotOrientation) {
						case NORTH:
							_robotOrientation = Orientation.SOUTH;
							break;
						case SOUTH:
							_robotOrientation = Orientation.NORTH;
							break;
						case EAST:
							_robotOrientation = Orientation.WEST;
							break;
						case WEST:
							_robotOrientation = Orientation.EAST;
							break;
						}
					}
				}

			} else if (hasAccessibleFront(_robotPosition, _robotOrientation)) {
				System.out.println("Right No Access, Accessible Front");
				_robot.moveForward();
				updateRobotPositionAfterMF(_robotOrientation, _robotPosition);
				setIsExplored(_robotPosition, _robotOrientation, true);
			} else {
				System.out.println("Right No Access, No Accessible Front");
				_robot.turnLeft();
				updateRobotOrientation(Movement.TURN_LEFT);
				setIsExplored(_robotPosition, _robotOrientation, true);
			}

		}
	}

	private void findImageAlongWall(int[] goalPos, int[] initial) {
		int move = 0;
		Controller controller = Controller.getInstance();
		PCClient pc = PCClient.getInstance();

		_robot.turnLeft();
		updateRobotOrientation(Movement.TURN_LEFT);

		while (!isGoalPos(_robotPosition, goalPos) && !controller.hasReachedTimeThreshold()) {
			int rightStatus = checkRightSide(_robotPosition, _robotOrientation);
			// System.out.println("Initial:"+initial[0]+","+initial[1]);
			eraseWall(_robotPosition, _robotOrientation);

			if (rightStatus != RIGHT_NO_ACCESS) {
				if (rightStatus == RIGHT_UNSURE_ACCESS) {
					_robot.turnRight();
					updateRobotOrientation(Movement.TURN_RIGHT);

					if (hasAccessibleFront(_robotPosition, _robotOrientation)) {
						_robot.moveForward();
						updateRobotPositionAfterMF(_robotOrientation, _robotPosition);
						move++;
						if (Arrays.equals(initial, _robotPosition) && move > 7) {
							return;
						}

					} else {
						_robot.turnLeft();
						updateRobotOrientation(Movement.TURN_LEFT);
					}
				} else { // rightStatus == RIGHT_CAN_ACCESS
					_robot.turnRight();
					updateRobotOrientation(Movement.TURN_RIGHT);

					_robot.moveForward();
					updateRobotPositionAfterMF(_robotOrientation, _robotPosition);
					move++;
					if (Arrays.equals(initial, _robotPosition) && move > 7) {
						return;
					}
				}

			} else if (hasAccessibleFront(_robotPosition, _robotOrientation)) {
				_robot.moveForward();
				updateRobotPositionAfterMF(_robotOrientation, _robotPosition);
				move++;
				if (Arrays.equals(initial, _robotPosition) && move > 7) {
					return;
				}
			} else {
				_robot.turnLeft();
				updateRobotOrientation(Movement.TURN_LEFT);
			}
		}
	}

	// tells me if my curPos is equal to my goalPos
	private boolean isGoalPos(int[] curPos, int[] goalPos) {
		if (curPos[0] == goalPos[0] && curPos[1] == goalPos[1]) {
			return true;
		}
		return false;
	}

	private void updateWall(int[] curPos, Orientation ori) {
		boolean hasObs = false;
		for (int i = -1; i < 2; i++) {
			switch (ori) {
			case NORTH:
				if ((curPos[0] + 2) != 15 && _mazeRef[curPos[0] + 2][curPos[1] + i] == IS_OBSTACLE) {
					rightWallRef[curPos[0] + 2][curPos[1] + i] = RIGHT_CHECK;
					hasObs = true;
				}
				break;
			case SOUTH:
				if ((curPos[0] - 2) != -1 && _mazeRef[curPos[0] - 2][curPos[1] + i] == IS_OBSTACLE) {
					rightWallRef[curPos[0] - 2][curPos[1] + i] = RIGHT_CHECK;
					hasObs = true;
				}
				break;
			case EAST:
				if ((curPos[1] - 2) != -1 && _mazeRef[curPos[0] + i][curPos[1] - 2] == IS_OBSTACLE) {
					rightWallRef[curPos[0] + i][curPos[1] - 2] = RIGHT_CHECK;
					hasObs = true;
				}
				break;
			case WEST:
				if ((curPos[1] + 2) != 20 && _mazeRef[curPos[0] + i][curPos[1] + 2] == IS_OBSTACLE) {
					rightWallRef[curPos[0] + i][curPos[1] + 2] = RIGHT_CHECK;
					hasObs = true;
				}
				break;
			}
		}

		if (hasObs) {
			try {
				sendObstaclePos(curPos, ori);
			} catch (IOException e1) {
				e1.printStackTrace();
			}
		}
	}

	private void eraseWall(int[] curPos, Orientation ori) {

		boolean hasObs = false;
		for (int i = -1; i < 2; i++) {
			switch (ori) {
			case NORTH:
				if ((curPos[0] + 2) != 15 && _mazeRef[curPos[0] + 2][curPos[1] + i] == IS_OBSTACLE) {
					imageRef[curPos[0] + 2][curPos[1] + i] = IS_EMPTY;
					hasObs = true;
				}
				break;
			case SOUTH:
				if ((curPos[0] - 2) != -1 && _mazeRef[curPos[0] - 2][curPos[1] + i] == IS_OBSTACLE) {
					imageRef[curPos[0] - 2][curPos[1] + i] = IS_EMPTY;
					hasObs = true;
				}
				break;
			case EAST:
				if ((curPos[1] - 2) != -1 && _mazeRef[curPos[0] + i][curPos[1] - 2] == IS_OBSTACLE) {
					imageRef[curPos[0] + i][curPos[1] - 2] = IS_EMPTY;
					hasObs = true;
				}
				break;
			case WEST:
				if ((curPos[1] + 2) != 20 && _mazeRef[curPos[0] + i][curPos[1] + 2] == IS_OBSTACLE) {
					imageRef[curPos[0] + i][curPos[1] + 2] = IS_EMPTY;
					hasObs = true;
				}
				break;
			}
		}

		if (hasObs && startImageRun) {
			try {
				sendObstaclePos(curPos, ori);
			} catch (IOException e1) {
				e1.printStackTrace();
			}
		}

	}

	private int checkRightSide(int[] curPos, Orientation ori) {
		int[] rightPos = new int[2];
		boolean hasUnexplored = false;

		switch (ori) {
		case NORTH:
			rightPos[0] = curPos[0] + 1;
			rightPos[1] = curPos[1];
			if (rightPos[0] + 1 >= Arena.MAP_LENGTH) {
				return RIGHT_NO_ACCESS;
			}
			for (int j = rightPos[1] - 1; j <= rightPos[1] + 1; j++) {
				if (_mazeRef[rightPos[0] + 1][j] == IS_OBSTACLE) {
					return RIGHT_NO_ACCESS;

				} else if (_mazeRef[rightPos[0] + 1][j] == UNEXPLORED) {
					hasUnexplored = true;
				}
			}
			break;
		case SOUTH:
			rightPos[0] = curPos[0] - 1;
			rightPos[1] = curPos[1];
			if (rightPos[0] - 1 < 0) {
				return RIGHT_NO_ACCESS;
			}
			for (int j = rightPos[1] - 1; j <= rightPos[1] + 1; j++) {
				if (_mazeRef[rightPos[0] - 1][j] == IS_OBSTACLE) {
					return RIGHT_NO_ACCESS;
				} else if (_mazeRef[rightPos[0] - 1][j] == UNEXPLORED) {
					hasUnexplored = true;
				}
			}
			break;
		case EAST:
			rightPos[0] = curPos[0];
			rightPos[1] = curPos[1] - 1;
			if (rightPos[1] - 1 < 0) {
				return RIGHT_NO_ACCESS;
			}
			for (int j = rightPos[0] - 1; j <= rightPos[0] + 1; j++) {
				if (_mazeRef[j][rightPos[1] - 1] == IS_OBSTACLE) {
					return RIGHT_NO_ACCESS;
				} else if (_mazeRef[j][rightPos[1] - 1] == UNEXPLORED) {
					hasUnexplored = true;
				}
			}
			break;
		case WEST:
			rightPos[0] = curPos[0];
			rightPos[1] = curPos[1] + 1;
			if (rightPos[1] + 1 >= Arena.MAP_WIDTH) {
				return RIGHT_NO_ACCESS;
			}
			for (int j = rightPos[0] - 1; j <= rightPos[0] + 1; j++) {
				if (_mazeRef[j][rightPos[1] + 1] == IS_OBSTACLE) {
					return RIGHT_NO_ACCESS;
				} else if (_mazeRef[j][rightPos[1] + 1] == UNEXPLORED) {
					hasUnexplored = true;
				}
			}
		}

		if (hasUnexplored) {
			return RIGHT_UNSURE_ACCESS;
		} else {
			return RIGHT_CAN_ACCESS;
		}
	}

	public boolean hasAccessibleFront(int[] curPos, Orientation ori) {
		int[] frontPos = new int[2];

		switch (ori) {
		case NORTH:
			frontPos[0] = curPos[0];
			frontPos[1] = curPos[1] + 1;
			if (frontPos[1] + 1 >= Arena.MAP_WIDTH) {
				return false;
			}
			for (int i = frontPos[0] - 1; i <= frontPos[0] + 1; i++) {
				if (_mazeRef[i][frontPos[1] + 1] == IS_OBSTACLE) {
					return false;
				}
			}
			return true;
		case SOUTH:
			frontPos[0] = curPos[0];
			frontPos[1] = curPos[1] - 1;
			if (frontPos[1] - 1 < 0) {
				return false;
			}
			for (int i = frontPos[0] - 1; i <= frontPos[0] + 1; i++) {
				if (_mazeRef[i][frontPos[1] - 1] == IS_OBSTACLE) {
					return false;
				}
			}
			return true;
		case EAST:
			frontPos[0] = curPos[0] + 1;
			frontPos[1] = curPos[1];
			if (frontPos[0] + 1 >= Arena.MAP_LENGTH) {
				return false;
			}
			for (int i = frontPos[1] - 1; i <= frontPos[1] + 1; i++) {
				if (_mazeRef[frontPos[0] + 1][i] == IS_OBSTACLE) {
					return false;
				}
			}
			return true;
		case WEST:
			frontPos[0] = curPos[0] - 1;
			frontPos[1] = curPos[1];
			if (frontPos[0] - 1 < 0) {
				return false;
			}
			for (int i = frontPos[1] - 1; i <= frontPos[1] + 1; i++) {
				if (_mazeRef[frontPos[0] - 1][i] == IS_OBSTACLE) {
					return false;
				}
			}
			return true;
		}
		return false;
	}

	private boolean canCalibrateAhead(String msgSensorValues) {
		Robot robot = Robot.getInstance();
		if (robot.getStepsSinceLastCalibration() == 0)
			return false;
		else
			return msgSensorValues.matches(Message.CALIBRATE_PATTERN);
	}

	private boolean canCalibrateAheadV2(String msgSensorValues) {
		Robot robot = Robot.getInstance();
		if (robot.getStepsSinceLastCalibration() == 0)
			return false;
		else if (msgSensorValues.matches(Message.CALIBRATE_PATTERN_CENTRE))
			return true;
		return false;
	}

	private Movement canCalibrateFront(int[] robotPosition, Orientation ori) {

		Robot robot = Robot.getInstance();
		if (robot.getStepsSinceLastCalibration() == 0)
			return null;
		else {

			int x = robotPosition[0];
			int y = robotPosition[1];
			switch (ori) {
			case NORTH:
				if (checkObstacleFront(x - 1, y + 2, Orientation.NORTH)
						&& checkObstacleFront(x + 1, y + 2, Orientation.NORTH))
					return Movement.LR;
				if (checkObstacleFront(x - 1, y + 2, Orientation.NORTH)
						&& checkObstacleFront(x, y + 2, Orientation.NORTH))
					return Movement.T;
				else if (checkObstacleFront(x, y + 2, Orientation.NORTH)
						&& checkObstacleFront(x + 1, y + 2, Orientation.NORTH))
					return Movement.Y;
				if (checkObstacleFront(x, y + 2, Orientation.NORTH))
					return Movement.M;
				else if (checkObstacleFront(x - 1, y + 2, Orientation.NORTH))
					return Movement.L;
				else if (checkObstacleFront(x + 1, y + 2, Orientation.NORTH))
					return Movement.R;
				break;

			case SOUTH:
				if (checkObstacleFront(x - 1, y - 2, Orientation.SOUTH)
						&& checkObstacleFront(x + 1, y - 2, Orientation.SOUTH))
					return Movement.LR;
				else if (checkObstacleFront(x + 1, y - 2, Orientation.NORTH)
						&& checkObstacleFront(x, y - 2, Orientation.NORTH))
					return Movement.T;
				else if (checkObstacleFront(x, y - 2, Orientation.NORTH)
						&& checkObstacleFront(x - 1, y - 2, Orientation.NORTH))
					return Movement.Y;
				if (checkObstacleFront(x, y - 2, Orientation.SOUTH))
					return Movement.M;
				else if (checkObstacleFront(x + 1, y - 2, Orientation.SOUTH))
					return Movement.L;
				else if (checkObstacleFront(x - 1, y - 2, Orientation.SOUTH))
					return Movement.R;
				break;

			case EAST:
				if (checkObstacleFront(x + 2, y + 1, Orientation.EAST)
						&& checkObstacleFront(x + 2, y - 1, Orientation.EAST))
					return Movement.LR;
				else if (checkObstacleFront(x + 2, y + 1, Orientation.NORTH)
						&& checkObstacleFront(x + 2, y, Orientation.NORTH))
					return Movement.T;
				else if (checkObstacleFront(x + 2, y, Orientation.NORTH)
						&& checkObstacleFront(x + 2, y - 1, Orientation.NORTH))
					return Movement.Y;
				if (checkObstacleFront(x + 2, y, Orientation.EAST))
					return Movement.M;
				else if (checkObstacleFront(x + 2, y + 1, Orientation.EAST))
					return Movement.L;
				else if (checkObstacleFront(x + 2, y - 1, Orientation.EAST))
					return Movement.R;
				break;
			case WEST:
				if (checkObstacleFront(x - 2, y + 1, Orientation.WEST)
						&& checkObstacleFront(x - 2, y - 1, Orientation.WEST))
					return Movement.LR;
				else if (checkObstacleFront(x - 2, y, Orientation.NORTH)
						&& checkObstacleFront(x - 2, y - 1, Orientation.NORTH))
					return Movement.T;
				else if (checkObstacleFront(x - 2, y, Orientation.NORTH)
						&& checkObstacleFront(x - 2, y + 1, Orientation.NORTH))
					return Movement.Y;
				if (checkObstacleFront(x - 2, y, Orientation.WEST))
					return Movement.M;
				else if (checkObstacleFront(x - 2, y - 1, Orientation.WEST))
					return Movement.L;
				else if (checkObstacleFront(x - 2, y + 1, Orientation.WEST))
					return Movement.R;
				break;
			}

			return null;
		}
	}

	private Movement canCalibrateFront2(int[] robotPosition, Orientation ori) {

		Robot robot = Robot.getInstance();
		if (robot.getStepsSinceLastCalibration() == 0)
			return null;
		else {

			int x = robotPosition[0];
			int y = robotPosition[1];
			switch (ori) {
			case NORTH:
				if (checkObstacleFront(x - 1, y + 3, Orientation.NORTH)
						&& checkObstacleFront(x + 1, y + 3, Orientation.NORTH))
					return Movement.LR;
				else if (checkObstacleFront(x, y + 3, Orientation.NORTH))
					return Movement.M;
				else if (checkObstacleFront(x - 1, y + 3, Orientation.NORTH))
					return Movement.L;
				else if (checkObstacleFront(x + 1, y + 3, Orientation.NORTH))
					return Movement.R;
				break;

			case SOUTH:
				if (checkObstacleFront(x - 1, y - 3, Orientation.SOUTH)
						&& checkObstacleFront(x + 1, y - 3, Orientation.SOUTH))
					return Movement.LR;
				else if (checkObstacleFront(x, y - 3, Orientation.SOUTH))
					return Movement.M;
				else if (checkObstacleFront(x + 1, y - 3, Orientation.SOUTH))
					return Movement.L;
				else if (checkObstacleFront(x - 1, y - 3, Orientation.SOUTH))
					return Movement.R;
				break;

			case EAST:
				if (checkObstacleFront(x + 3, y + 1, Orientation.EAST)
						&& checkObstacleFront(x + 3, y - 1, Orientation.EAST))
					return Movement.LR;
				else if (checkObstacleFront(x + 3, y, Orientation.EAST))
					return Movement.M;
				else if (checkObstacleFront(x + 3, y + 1, Orientation.EAST))
					return Movement.L;
				else if (checkObstacleFront(x + 3, y - 1, Orientation.EAST))
					return Movement.R;
				break;
			case WEST:
				if (checkObstacleFront(x - 3, y + 1, Orientation.WEST)
						&& checkObstacleFront(x - 3, y - 1, Orientation.WEST))
					return Movement.LR;
				else if (checkObstacleFront(x - 3, y, Orientation.WEST))
					return Movement.M;
				else if (checkObstacleFront(x - 3, y - 1, Orientation.WEST))
					return Movement.L;
				else if (checkObstacleFront(x - 3, y + 1, Orientation.WEST))
					return Movement.R;
				break;
			}

			return null;
		}
	}
	/**
	 * 0 - cant calibrate
	 * 1 - calibrate front
	 * 2 - calibrate right
	 */
	public int canCalibrateFront3(Orientation _ori)
	{
		Robot robot = Robot.getInstance();
		int _x = _robotPosition[0];
		int _y = _robotPosition[1];
		if (robot.getStepsSinceLastCalibration() == 0)
			return 0;
		switch (_ori)
		{
		case NORTH:
			if (_y == Arena.MAP_WIDTH-2 || (_mazeRef[_x][_y+2] == IS_OBSTACLE && _mazeRef[_x-1][_y+2] == IS_OBSTACLE && _mazeRef[_x+1][_y+2] == IS_OBSTACLE))
				return 1;
			else if (_x == Arena.MAP_LENGTH-2 || (_mazeRef[_x+2][_y] == IS_OBSTACLE && _mazeRef[_x+2][_y-1] == IS_OBSTACLE && _mazeRef[_x+2][_y+1] == IS_OBSTACLE))
				return 2;
			break;
		case SOUTH:
			if (_y == 1 || (_mazeRef[_x][_y-2] == IS_OBSTACLE && _mazeRef[_x-1][_y-2] == IS_OBSTACLE && _mazeRef[_x+1][_y-2] == IS_OBSTACLE))
				return 1;
			else if (_x == 1 || (_mazeRef[_x-2][_y] == IS_OBSTACLE && _mazeRef[_x-2][_y-1] == IS_OBSTACLE && _mazeRef[_x-2][_y+1] == IS_OBSTACLE))
				return 2;		
			break;
		case EAST:
			if (_x == Arena.MAP_LENGTH-2 || (_mazeRef[_x+2][_y] == IS_OBSTACLE && _mazeRef[_x+2][_y-1] == IS_OBSTACLE && _mazeRef[_x+2][_y+1] == IS_OBSTACLE))
				return 1;
			else if (_y == 1 || (_mazeRef[_x][_y-2] == IS_OBSTACLE && _mazeRef[_x-1][_y-2] == IS_OBSTACLE && _mazeRef[_x+1][_y-2] == IS_OBSTACLE))
				return 2;	
			break;
		case WEST:
			if (_x == 1 || (_mazeRef[_x-2][_y] == IS_OBSTACLE && _mazeRef[_x-2][_y-1] == IS_OBSTACLE && _mazeRef[_x-2][_y+1] == IS_OBSTACLE))
				return 1;
			else if (_y == Arena.MAP_WIDTH-2 || (_mazeRef[_x][_y+2] == IS_OBSTACLE && _mazeRef[_x-1][_y+2] == IS_OBSTACLE && _mazeRef[_x+1][_y+2] == IS_OBSTACLE))
				return 2;
			break;
		}
		return 0;
	}
	public void canTakePicFront(Orientation _ori)
	{
		Robot robot = Robot.getInstance();
		int _x = _robotPosition[0];
		int _y = _robotPosition[1];
		
		boolean doITakePic = false;
		ImageRef imageRef = new ImageRef();
		switch (_ori)
		{
		case NORTH:
			if (_y < Arena.MAP_WIDTH-2)
			{
				if (_mazeRef[_x][_y+2] == IS_OBSTACLE || _mazeRef[_x-1][_y+2] == IS_OBSTACLE || _mazeRef[_x+1][_y+2] == IS_OBSTACLE)
				{
					imageRef.setTargetX(_x);
					imageRef.setTargetY(_y+2);
					doITakePic = true;
				}
				else if (_y < Arena.MAP_WIDTH-3 && (_mazeRef[_x][_y+3] == IS_OBSTACLE || _mazeRef[_x-1][_y+3] == IS_OBSTACLE || _mazeRef[_x+1][_y+3] == IS_OBSTACLE))
				{
					imageRef.setTargetX(_x);
					imageRef.setTargetY(_y+3);
					doITakePic = true;
				}
				imageRef.setOrientation(Orientation.SOUTH);
			}
				break;
		case SOUTH:
			if (_y > 1)		
			{
				if (_mazeRef[_x][_y-2] == IS_OBSTACLE || _mazeRef[_x-1][_y-2] == IS_OBSTACLE || _mazeRef[_x+1][_y-2] == IS_OBSTACLE)
				{
					imageRef.setTargetX(_x);
					imageRef.setTargetY(_y-2);
					doITakePic = true;
				}
				else if (_y > 2 && (_mazeRef[_x][_y-3] == IS_OBSTACLE || _mazeRef[_x-1][_y-3] == IS_OBSTACLE || _mazeRef[_x+1][_y-3] == IS_OBSTACLE))
				{
					imageRef.setTargetX(_x);
					imageRef.setTargetY(_y-3);
					doITakePic = true;
				}
				imageRef.setOrientation(Orientation.NORTH);
			}
				break;
		case EAST:
			if (_x < Arena.MAP_LENGTH-2)
			{
				if (_mazeRef[_x+2][_y] == IS_OBSTACLE || _mazeRef[_x+2][_y-1] == IS_OBSTACLE || _mazeRef[_x+2][_y+1] == IS_OBSTACLE)
				{
					imageRef.setTargetX(_x+2);
					imageRef.setTargetY(_y);	
					doITakePic = true;
				}
				else if (_x < Arena.MAP_LENGTH-3 && (_mazeRef[_x+3][_y] == IS_OBSTACLE || _mazeRef[_x+3][_y-1] == IS_OBSTACLE || _mazeRef[_x+3][_y+1] == IS_OBSTACLE))
				{
					imageRef.setTargetX(_x+3);
					imageRef.setTargetY(_y);	
					doITakePic = true;
				}
				imageRef.setOrientation(Orientation.WEST);
			}	
				break;
		case WEST:
			if (_x > 1)
			{
				if (_mazeRef[_x-2][_y] == IS_OBSTACLE || _mazeRef[_x-2][_y-1] == IS_OBSTACLE || _mazeRef[_x-2][_y+1] == IS_OBSTACLE)
				{
					imageRef.setTargetX(_x-2);
					imageRef.setTargetY(_y);
					doITakePic = true;
				}
				else if (_x > 2 && (_mazeRef[_x-3][_y] == IS_OBSTACLE || _mazeRef[_x-3][_y-1] == IS_OBSTACLE || _mazeRef[_x-3][_y+1] == IS_OBSTACLE))
				{
					imageRef.setTargetX(_x-3);
					imageRef.setTargetY(_y);
					doITakePic = true;
				}
			}
			imageRef.setOrientation(Orientation.EAST);
				break;
		}
		
		if (doITakePic)
		{
			imageRef.setX(_x);
			imageRef.setY(_y);
			imageRef.setOrientation(_robotOrientation);
			
			arrayListOfImageRefsExploration.add(imageRef);
			System.out.println("taking pic now");
			sendPicToRPI(imageRef);
		}		
	}
	
	private Movement canCalibrateLeft(int[] robotPosition, Orientation ori) {
		int x = robotPosition[0];
		int y = robotPosition[1];
		switch (ori) {
		case NORTH:
			if (checkObstacleLeft(x - 2, y + 1, Orientation.NORTH)
					&& checkObstacleLeft(x - 2, y - 1, Orientation.NORTH))
				return Movement.LR;
			else if (checkObstacleLeft(x - 2, y, Orientation.NORTH)
					&& checkObstacleLeft(x - 2, y - 1, Orientation.NORTH))
				return Movement.T;
			else if (checkObstacleLeft(x - 2, y, Orientation.NORTH)
					&& checkObstacleLeft(x - 2, y + 1, Orientation.NORTH))
				return Movement.Y;
			else if (checkObstacleLeft(x - 2, y, Orientation.NORTH))
				return Movement.M;
			else if (checkObstacleLeft(x - 2, y - 1, Orientation.NORTH))
				return Movement.L;
			else if (checkObstacleLeft(x - 2, y + 1, Orientation.NORTH))
				return Movement.R;
			break;
		case SOUTH:
			if (checkObstacleLeft(x + 2, y + 1, Orientation.SOUTH)
					&& checkObstacleLeft(x + 2, y - 1, Orientation.SOUTH))
				return Movement.LR;
			else if (checkObstacleLeft(x + 2, y + 1, Orientation.SOUTH)
					&& checkObstacleLeft(x + 2, y, Orientation.SOUTH))
				return Movement.T;
			else if (checkObstacleLeft(x + 2, y, Orientation.SOUTH)
					&& checkObstacleLeft(x + 2, y - 1, Orientation.SOUTH))
				return Movement.Y;
			else if (checkObstacleLeft(x + 2, y, Orientation.SOUTH))
				return Movement.M;
			else if (checkObstacleLeft(x + 2, y + 1, Orientation.SOUTH))
				return Movement.L;
			else if (checkObstacleLeft(x + 2, y - 1, Orientation.SOUTH))
				return Movement.R;
			break;
		case EAST:
			if (checkObstacleLeft(x - 1, y + 2, Orientation.EAST) && checkObstacleLeft(x + 1, y + 2, Orientation.EAST))
				return Movement.LR;
			else if (checkObstacleLeft(x - 1, y + 2, Orientation.EAST) && checkObstacleLeft(x, y + 2, Orientation.EAST))
				return Movement.T;
			else if (checkObstacleLeft(x, y + 2, Orientation.EAST) && checkObstacleLeft(x + 1, y + 2, Orientation.EAST))
				return Movement.Y;
			else if (checkObstacleLeft(x, y + 2, Orientation.EAST))
				return Movement.M;
			else if (checkObstacleLeft(x - 1, y + 2, Orientation.EAST))
				return Movement.L;
			else if (checkObstacleLeft(x + 1, y + 2, Orientation.EAST))
				return Movement.R;
			break;
		case WEST:

			if (checkObstacleLeft(x - 1, y - 2, Orientation.WEST) && checkObstacleLeft(x + 1, y - 2, Orientation.WEST))
				return Movement.LR;
			else if (checkObstacleLeft(x + 1, y - 2, Orientation.WEST) && checkObstacleLeft(x, y - 2, Orientation.WEST))
				return Movement.T;
			else if (checkObstacleLeft(x, y - 2, Orientation.WEST) && checkObstacleLeft(x - 1, y - 2, Orientation.WEST))
				return Movement.Y;
			else if (checkObstacleLeft(x, y - 2, Orientation.WEST))
				return Movement.M;
			else if (checkObstacleLeft(x + 1, y - 2, Orientation.WEST))
				return Movement.L;
			else if (checkObstacleLeft(x - 1, y - 2, Orientation.WEST))
				return Movement.R;
			break;
		}

		return null;
	}

	private boolean isDeadEnd(int[] robotPosition, Orientation ori) {
		int x = robotPosition[0];
		int y = robotPosition[1];
		switch (ori) {
		case NORTH:
			if ((checkObstacleFront(x - 1, y + 2, Orientation.NORTH) || checkObstacleFront(x, y + 2, Orientation.NORTH)
					|| checkObstacleFront(x + 1, y + 2, Orientation.NORTH))
					&& (checkObstacleLeft(x - 2, y + 1, Orientation.NORTH)
							|| checkObstacleLeft(x - 2, y, Orientation.NORTH)
							|| checkObstacleLeft(x - 2, y - 1, Orientation.NORTH))
					&& (checkObstacle(x + 2, y + 1, Orientation.NORTH) || checkObstacle(x + 2, y, Orientation.NORTH)
							|| checkObstacle(x + 2, y - 1, Orientation.NORTH)))
				return true;
			break;
		case SOUTH:
			if ((checkObstacle(x - 2, y + 1, Orientation.SOUTH) || checkObstacle(x - 2, y, Orientation.SOUTH)
					|| checkObstacle(x - 2, y - 1, Orientation.SOUTH))
					&& (checkObstacleLeft(x + 2, y + 1, Orientation.SOUTH)
							|| checkObstacleLeft(x + 2, y, Orientation.SOUTH)
							|| checkObstacleLeft(x + 2, y - 1, Orientation.SOUTH))
					&& (checkObstacleFront(x - 1, y - 2, Orientation.SOUTH)
							|| checkObstacleFront(x, y - 2, Orientation.SOUTH)
							|| checkObstacleFront(x + 1, y - 2, Orientation.SOUTH)))
				return true;
			break;
		case EAST:
			if ((checkObstacleLeft(x - 1, y + 2, Orientation.EAST) || checkObstacleLeft(x, y + 2, Orientation.EAST)
					|| checkObstacleLeft(x + 1, y + 2, Orientation.EAST))
					&& (checkObstacleFront(x + 2, y + 1, Orientation.EAST)
							|| checkObstacleFront(x + 2, y, Orientation.EAST)
							|| checkObstacleFront(x + 2, y - 1, Orientation.EAST))
					&& (checkObstacle(x - 1, y - 2, Orientation.EAST) || checkObstacle(x, y - 2, Orientation.EAST)
							|| checkObstacle(x + 1, y - 2, Orientation.EAST)))
				return true;
			break;
		case WEST:
			if ((checkObstacle(x - 1, y + 2, Orientation.WEST) || checkObstacle(x, y + 2, Orientation.WEST)
					|| checkObstacle(x + 1, y + 2, Orientation.WEST))
					&& (checkObstacleFront(x - 2, y + 1, Orientation.WEST)
							|| checkObstacleFront(x - 2, y, Orientation.WEST)
							|| checkObstacleFront(x - 2, y - 1, Orientation.WEST))
					&& (checkObstacleLeft(x - 1, y - 2, Orientation.WEST)
							|| checkObstacleLeft(x, y - 2, Orientation.WEST)
							|| checkObstacleLeft(x + 1, y - 2, Orientation.WEST)))
				return true;
			break;
		}

		return false;
	}

	private Movement canCalibrateRight(int[] robotPosition, Orientation ori) {
		int x = robotPosition[0];
		int y = robotPosition[1];
		switch (ori) {
		case NORTH:
			if (checkObstacle(x + 2, y + 1, Orientation.NORTH) && checkObstacle(x + 2, y - 1, Orientation.NORTH))
				return Movement.LR;
			else if (checkObstacle(x + 2, y + 1, Orientation.NORTH) && checkObstacle(x + 2, y, Orientation.NORTH))
				return Movement.T;
			else if (checkObstacle(x + 2, y, Orientation.NORTH) && checkObstacle(x + 2, y - 1, Orientation.NORTH))
				return Movement.Y;
			else if (checkObstacle(x + 2, y, Orientation.NORTH))
				return Movement.M;
			else if (checkObstacle(x + 2, y + 1, Orientation.NORTH))
				return Movement.L;
			else if (checkObstacle(x + 2, y - 1, Orientation.NORTH))
				return Movement.R;
			break;
		case SOUTH:
			if (checkObstacle(x - 2, y + 1, Orientation.SOUTH) && checkObstacle(x - 2, y - 1, Orientation.SOUTH))
				return Movement.LR;
			else if (checkObstacle(x - 2, y, Orientation.SOUTH) && checkObstacle(x - 2, y - 1, Orientation.SOUTH))
				return Movement.T;
			else if (checkObstacle(x - 2, y, Orientation.SOUTH) && checkObstacle(x - 2, y + 1, Orientation.SOUTH))
				return Movement.Y;
			else if (checkObstacle(x - 2, y, Orientation.SOUTH))
				return Movement.M;
			else if (checkObstacle(x - 2, y - 1, Orientation.SOUTH))
				return Movement.L;
			else if (checkObstacle(x - 2, y + 1, Orientation.SOUTH))
				return Movement.R;
			break;
		case EAST:
			if (checkObstacle(x - 1, y - 2, Orientation.EAST) && checkObstacle(x + 1, y - 2, Orientation.EAST))
				return Movement.LR;
			else if (checkObstacle(x + 1, y - 2, Orientation.EAST) && checkObstacle(x, y - 2, Orientation.EAST))
				return Movement.T;
			else if (checkObstacle(x, y - 2, Orientation.EAST) && checkObstacle(x - 1, y - 2, Orientation.EAST))
				return Movement.Y;
			else if (checkObstacle(x, y - 2, Orientation.EAST))
				return Movement.M;
			else if (checkObstacle(x + 1, y - 2, Orientation.EAST))
				return Movement.L;
			else if (checkObstacle(x - 1, y - 2, Orientation.EAST))
				return Movement.R;
			break;
		case WEST:
			if (checkObstacle(x - 1, y + 2, Orientation.WEST) && checkObstacle(x + 1, y + 2, Orientation.WEST))
				return Movement.LR;
			else if (checkObstacle(x - 1, y + 2, Orientation.WEST) && checkObstacle(x, y + 2, Orientation.WEST))
				return Movement.T;
			else if (checkObstacle(x, y + 2, Orientation.WEST) && checkObstacle(x + 1, y + 2, Orientation.WEST))
				return Movement.Y;
			else if (checkObstacle(x, y + 2, Orientation.WEST))
				return Movement.M;
			else if (checkObstacle(x - 1, y + 2, Orientation.WEST))
				return Movement.L;
			else if (checkObstacle(x + 1, y + 2, Orientation.WEST))
				return Movement.R;
			break;
		}

		return null;
	}

	private boolean checkObstacleFront(int x, int y, Orientation ori) {
		switch (ori) {
		case NORTH:
			if (y == 20 || _mazeRef[x][y] == IS_OBSTACLE)
				return true;
			break;
		case SOUTH:
			if (y == -1 || _mazeRef[x][y] == IS_OBSTACLE)
				return true;
			break;
		case EAST:
			if (x == 15 || _mazeRef[x][y] == IS_OBSTACLE)
				return true;
			break;
		case WEST:
			if (x == -1 || _mazeRef[x][y] == IS_OBSTACLE)
				return true;
			break;
		}
		return false;
	}

	private boolean checkObstacle(int x, int y, Orientation ori) {
		switch (ori) {
		case NORTH:
			if (x == 15 || _mazeRef[x][y] == IS_OBSTACLE)
				return true;
			break;
		case SOUTH:
			if (x == -1 || _mazeRef[x][y] == IS_OBSTACLE)
				return true;
			break;
		case EAST:
			if (y == -1 || _mazeRef[x][y] == IS_OBSTACLE)
				return true;
			break;
		case WEST:
			if (y == 20 || _mazeRef[x][y] == IS_OBSTACLE)
				return true;
			break;
		}
		return false;
	}

	private boolean checkObstacleLeft(int x, int y, Orientation ori) {
		switch (ori) {
		case NORTH:
			if (x == -1 || _mazeRef[x][y] == IS_OBSTACLE)
				return true;
			break;
		case SOUTH:
			if (x == 15 || _mazeRef[x][y] == IS_OBSTACLE)
				return true;
			break;
		case EAST:
			if (y == 20 || _mazeRef[x][y] == IS_OBSTACLE)
				return true;
			break;
		case WEST:
			if (y == -1 || _mazeRef[x][y] == IS_OBSTACLE)
				return true;
			break;
		}
		return false;
	}

	private Movement canCalibrateAside(int[] robotPosition, Orientation ori) {

		boolean leftHasRef = true, rightHasRef = true;
		Movement move = null;
		switch (ori) {
		case NORTH:
			if (robotPosition[0] - 2 == -1) {
				move = Movement.TURN_LEFT;
			} else if (robotPosition[0] + 2 == Arena.MAP_LENGTH) {
				move = Movement.TURN_RIGHT;
			} else {
				for (int i = -1; i <= 1; i++) {
					if (_mazeRef[robotPosition[0] - 2][robotPosition[1] + i] != IS_OBSTACLE) {
						leftHasRef = false;
					}
					if (_mazeRef[robotPosition[0] + 2][robotPosition[1] + i] != IS_OBSTACLE) {
						rightHasRef = false;
					}
				}
				if (leftHasRef) {
					move = Movement.TURN_LEFT;
				} else if (rightHasRef) {
					move = Movement.TURN_RIGHT;
				}
			}
			break;
		case SOUTH:
			if (robotPosition[0] - 2 == -1) {
				move = Movement.TURN_RIGHT;
			} else if (robotPosition[0] + 2 == Arena.MAP_LENGTH) {
				move = Movement.TURN_LEFT;
			} else {
				for (int i = -1; i <= 1; i++) {
					if (_mazeRef[robotPosition[0] - 2][robotPosition[1] + i] != IS_OBSTACLE) {
						rightHasRef = false;
					}
					if (_mazeRef[robotPosition[0] + 2][robotPosition[1] + i] != IS_OBSTACLE) {
						leftHasRef = false;
					}
				}
				if (leftHasRef) {
					move = Movement.TURN_LEFT;
				} else if (rightHasRef) {
					move = Movement.TURN_RIGHT;
				}
			}
			break;
		case EAST:
			if (robotPosition[1] - 2 == -1) {
				move = Movement.TURN_RIGHT;
			} else if (robotPosition[1] + 2 == Arena.MAP_WIDTH) {
				move = Movement.TURN_LEFT;
			} else {
				for (int i = -1; i <= 1; i++) {
					if (_mazeRef[robotPosition[0] + i][robotPosition[1] - 2] != IS_OBSTACLE) {
						rightHasRef = false;
					}
					if (_mazeRef[robotPosition[0] + i][robotPosition[1] + 2] != IS_OBSTACLE) {
						leftHasRef = false;
					}
				}
				if (leftHasRef) {
					move = Movement.TURN_LEFT;
				} else if (rightHasRef) {
					move = Movement.TURN_RIGHT;
				}
			}
			break;
		case WEST:
			if (robotPosition[1] - 2 == -1) {
				move = Movement.TURN_LEFT;
			} else if (robotPosition[1] + 2 == Arena.MAP_WIDTH) {
				move = Movement.TURN_RIGHT;
			} else {
				for (int i = -1; i <= 1; i++) {
					if (_mazeRef[robotPosition[0] + i][robotPosition[1] - 2] != IS_OBSTACLE) {
						leftHasRef = false;
					}
					if (_mazeRef[robotPosition[0] + i][robotPosition[1] + 2] != IS_OBSTACLE) {
						rightHasRef = false;
					}
				}
				if (leftHasRef) {
					move = Movement.TURN_LEFT;
				} else if (rightHasRef) {
					move = Movement.TURN_RIGHT;
				}
			}
		}
		return move;
	}

	public static int parseSensorValue(String msgSensorValues, SensorPosition sensorPosition) {
		// String sensorValues = msgSensorValues.substring(0, msgSensorValues.length() -
		// 1);
		List<String> valueList = Arrays.asList(msgSensorValues.split(":"));
		// Left Short/Left Long : Right Short : Front Short : Front Left Short : Front
		// Right Short
		// System.out.println("LF CF RF L R R2");
		// System.out.println("sensor values: " + valueList);

		int reading = INVALID_SENSOR_VALUE;

		switch (sensorPosition) {
		case LF:
			reading = Integer.parseInt(valueList.get(0));
			// System.out.println("LF " + reading);
			break;

		case CF:
			reading = Integer.parseInt(valueList.get(1));
			// System.out.println(reading);
			// System.out.println("CF " + reading);
			break;

		case RF:
			reading = Integer.parseInt(valueList.get(2));
			// System.out.println(reading);
			// System.out.println("RF " + reading);
			break;

		case L:
			reading = Integer.parseInt(valueList.get(3));
			// System.out.println(reading);
			// System.out.println("L " + reading);
			break;

		case R:
			reading = Integer.parseInt(valueList.get(4));
			// System.out.println(reading);
			// System.out.println("R " + reading);
			break;

		case R2:
			char interim = valueList.get(5).charAt(0);
			reading = Integer.parseInt(String.valueOf(interim));
			// System.out.println("R2 " + reading);
			break;
		}

		return reading;

	}
}