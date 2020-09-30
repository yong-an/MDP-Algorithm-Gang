package simulator;

import java.awt.CardLayout;
import java.awt.Color;
import java.awt.Toolkit;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.IOException;
import java.net.UnknownHostException;
import java.nio.file.FileSystems;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JPanel;
import javax.swing.SwingWorker;
import javax.swing.Timer;

import algorithms.AStarPathFinder;
import algorithms.MazeExplorer;
import algorithms.Path;
import datatypes.Message;
import datatypes.Orientation;
import main.RobotSystem;
import simulator.arena.Arena;
import simulator.arena.FileReaderWriter;
import simulator.robot.Robot;
import tcpcomm.PCClient;
/**
 * This Java file handles the simulator UI logic / real run logic -> depending on RPI whether is it connected or not.
 */
public class Controller {

	public static final String ARENA_DESCRIPTOR_PATH = System.getProperty("user.dir") + "/map-descriptors/Map-Descriptor-Output.txt";
	private static final int THRESHOLD_BUFFER_TIME = 10;
	private static final int EXPLORE_TIME_LIMIT = 360;
	private static final int FFP_TIME_LIMIT = 120;
	private static final int EXPLORE_REAL_RUN_SPEED = 1;
	private static final int EXPLORE_COVERAGE = 100;
	
	private static Controller _instance;
	private UI _ui;
	private Timer _exploreTimer, _ffpTimer;
	private int[] _robotPosition = new int[2];
	private int[] _waypointPosition = new int[2];
	private Orientation _robotOrientation;
	private int _speed,  _exploreTimeLimit, _ffpTimeLimit;
	private float _actualCoverage, _initialCoverage;
	private boolean _hasReachedStart, _hasReachedTimeThreshold;
	private PCClient _pcClient;
	private Path _fastestPath, _fastestPath2, _fastestBackPath;

    /**
     * This function will generate the UI.
     */
    private Controller() {
        _ui = new UI();
    }

    /**
     * This function will generate a controller instance.
     *
     * @return
     */
    public static Controller getInstance() {
        if (_instance == null) {
            _instance = new Controller();
        }
        return _instance;
        
    /**
    * This function returns the robot orientation
    *
    * @return
    */
    }
	public Orientation getOrientation() {
		return this._robotOrientation;
	}
	
    /**
    * This function returns robot position in array format
    *
    * @return
    */
	public int[] getPosition() {
		return this._robotPosition;
	}
	
    /**
    * This function returns the fastest path
    *
    * @return
    */
	public Path getFastestPathBack() {
		return _fastestBackPath;
	}

    /**
     * This function handles the run logic.
     * It check if it's a real run or just a simulated run.
     * And display the UI accordingly.
     */
	public void run() {
		_ui.setVisible(true);
		_robotOrientation = Orientation.EAST;
		if (RobotSystem.isRealRun()) {
	
			_pcClient = PCClient.getInstance();
			
			SwingWorker<Void, Void> connectWithRPi = new SwingWorker<Void, Void>() {
			
				@Override
				protected Void doInBackground() throws Exception {
					
					try {
						//Connect with RPi
						_ui.setStatus("waiting for connection...");
						_pcClient.setUpConnection(PCClient.RPI_IP_ADDRESS, PCClient.RPI_PORT);
						_ui.setStatus("connected with RPi");
						
						//Read initial robot position
						String msgRobotPosition = "";
						String msgWayPoint = "";
						boolean receivePosition = true;
						boolean receiveWaypoint = true;
						String messageReceive = "";
						List<String> valueList;
						
						/*while(receivePosition || receiveWaypoint) {
							messageReceive = _pcClient.readMessage();
							valueList = Arrays.asList(messageReceive.split(":"));
							
							if(receivePosition && valueList.get(0).equals("START")) {
								msgRobotPosition = valueList.get(1);
								int[] robotPosInput = getRobotPositionInput(msgRobotPosition);
								resetRobotInMaze(_ui.getMazeGrids(), robotPosInput[0], robotPosInput[1]);
								receivePosition = false;
							}
							if(receiveWaypoint && valueList.get(0).equals("WP")) {
								msgWayPoint = valueList.get(1);
								setWayPointInMaze(_ui.getMazeGrids(), msgWayPoint);
								receiveWaypoint = false;
							}
						}*/
						
						msgRobotPosition = "1,1";
						//   msgRobotPosition = valueList.get(1);
						int[] robotPosInput = getRobotPositionInput(msgRobotPosition);
						resetRobotInMaze(_ui.getMazeGrids(), robotPosInput[0], robotPosInput[1]);

						msgWayPoint = "6,18";
						//   String msgWayPoint = valueList.get(1);
						setWayPointInMaze(_ui.getMazeGrids(), msgWayPoint);
						
				
						MazeExplorer me = MazeExplorer.getInstance();
						me.init(_robotPosition, _robotOrientation);
						Robot caliRobot = Robot.getInstance();
						
						_robotOrientation = caliRobot.calibrateAtStartZone(_robotOrientation);
						me.setOrientation(_robotOrientation);
						
						String msgExplore = _pcClient.readMessage();
						//String msgExplore = Message.START_EXPLORATION;
						while (!msgExplore.equals(Message.START_EXPLORATION)) {
							msgExplore = _pcClient.readMessage();
						}

						_ui.setStatus("start exploring");
						exploreMaze();

				} catch (UnknownHostException e) {
					e.printStackTrace();
					System.out.println(e.toString());
				} catch (IOException e) {
					e.printStackTrace();
				}
					return null;
				}
				
			};
			
			connectWithRPi.execute();

	
		} else {
			_ui.refreshExploreInput();
		}
	}
	
    /**
     * Function will return the robot position in descriptor format.
     *
     * @return
     */
	public int[] getRobotPositionInput (String msgRobotPosition) {
		int[] posInput = new int[2];
		int index = msgRobotPosition.indexOf(",");
		posInput[0] = Integer.parseInt(msgRobotPosition.substring(0, index));
		posInput[1] = Integer.parseInt(msgRobotPosition.substring(index + 1));
		return posInput;
	}

    /**
     * Function will return the UI.
     *
     * @return
     */
    public UI getUI() {
        return _ui;
    }

    /**
     * Function will return the PC client.
     *
     * @return
     */
    public PCClient getPCClient() {
        return _pcClient;
    }

    /**
     * Function to let you know if you ran out of time.
     *
     * @return
     */
    public boolean hasReachedTimeThreshold() {
    	System.out.println("Reach Time Threshold: "+_hasReachedTimeThreshold);
        return _hasReachedTimeThreshold;
    }

    /**
     * Function to handle the toggling of grids.
     * White = no obstacle.
     * Red = obstacle.
     *
     * @param mapGrids
     * @param x
     * @param y
     */
    public void toggleObstacle(JButton[][] mapGrids, int x, int y) {
        if (mapGrids[x][y].getBackground() == Color.WHITE) {
            mapGrids[x][y].setBackground(Color.RED);
        } else {
            mapGrids[x][y].setBackground(Color.WHITE);
        }
    }

    /**
     * Function to handle the logic behind switching of 2 different layout for the middle panel.
     *
     * @param cb
     * @param cardPanel
     */
    public void switchComboBox(JComboBox cb, JPanel cardPanel) {
        CardLayout cardLayout = (CardLayout) (cardPanel.getLayout());
        cardLayout.show(cardPanel, (String) cb.getSelectedItem());
    }

    /**
     * Function to handle to logic behind load map button.
     *
     * @param mapGrids
     */
    public void loadMap(JButton[][] mapGrids) {
        Arena arena = Arena.getInstance();
        arena.setLayout(mapGrids);

        Boolean[][] layout = arena.getLayout();
        String arenaDescriptor = "";

        for (int i = 0; i < Arena.MAP_WIDTH; i++) {
            for (int j = 0; j < Arena.MAP_LENGTH; j++) {
                if (layout[j][i] == true) {
                    arenaDescriptor += "1";
                } else {
                    arenaDescriptor += "0";
                }
            }
            arenaDescriptor = arenaDescriptor + System.getProperty("line.separator");
        }

        try {
            FileReaderWriter fileWriter = new FileReaderWriter(FileSystems.getDefault().getPath(ARENA_DESCRIPTOR_PATH));
            fileWriter.write(arenaDescriptor);
        } catch (IOException e) {
            e.printStackTrace();
        }

        _ui.setStatus("finished map loading, ready to explore");
    }

    /**
     * Function to handle to logic behind clear map button.
     *
     * @param mapGrids
     */
    public void clearMap(JButton[][] mapGrids) {
        for (int x = 0; x < Arena.MAP_WIDTH; x++) {
            for (int y = 0; y < Arena.MAP_LENGTH; y++) {
                if (mapGrids[x][y].getBackground() == Color.RED) {
                    mapGrids[x][y].setBackground(Color.WHITE);
                }
            }
        }

        Arena arena = Arena.getInstance();
        arena.setLayout(mapGrids);

        _ui.setStatus("finished map clearing");
    }

    /**
     * Function to reset the simulator robot position if it somehow run out of scope.
     *
     * @param mazeGrids
     * @param x
     * @param y
     */
    public void resetRobotInMaze(JButton[][] mazeGrids, int x, int y) {
		if(RobotSystem.isRealRun()) {
			x+=1;
			y+=1;
		}
		if (x < 2 || x > 14 || y < 2 || y > 9) {
			_ui.setStatus("warning: robot position out of range");
			resetMaze(mazeGrids);
		} else {
			for (int i = x - 1; i <= x + 1; i++) {
				for (int j = y - 1; j <= y + 1; j++) {
					mazeGrids[20 - j][i - 1].setBackground(Color.GREEN);		
				}
			}
			
			//_robotOrientation = Orientation.NORTH;
			
			switch (_robotOrientation) {
			case NORTH:
				mazeGrids[19 - y][x - 1].setBackground(Color.BLUE);	
				break;
			case SOUTH:
				mazeGrids[21 - y][x - 1].setBackground(Color.BLUE);	
				break;
			case EAST:
				mazeGrids[20 - y][x].setBackground(Color.BLUE);	
				break;
			case WEST:
				mazeGrids[20 - y][x - 2].setBackground(Color.BLUE);	
				break;
			}
			
			_robotPosition[0] = x - 1;
			_robotPosition[1] = y - 1;
		
			System.out.println("Robot initial position set at "+_robotPosition[0]+","+_robotPosition[1]);
			_ui.setStatus("robot initial position set");
		}
	}
    
    /**
     * Function to set way point in maze.
     *
     * @param mazeGrids
     * @param waypoint
     */
    public void setWayPointInMaze(JButton[][] mazeGrids, String waypoint) {
		
		if((_waypointPosition[1] != 0 && _waypointPosition[0] != 0) || waypoint.isEmpty())
			mazeGrids[20-(_waypointPosition[1]+1)][(_waypointPosition[0]+1)-1].setText("");	
		if(!waypoint.isEmpty()) {
			int[] mazeWayPoint = getRobotPositionInput(waypoint);
			int x = mazeWayPoint[0];
			int y = mazeWayPoint[1];
			
			if(RobotSystem.isRealRun()) {
				x+=1;
				y+=1;
			}
			
			if (x < 2 || x > 14 || y < 2 || y > 19) {
				_ui.setStatus("warning: robot position out of range");
				resetMaze(mazeGrids);
			} else {
				mazeGrids[20-y][x - 1].setText("WP");	
				_waypointPosition[0] = x - 1;
				_waypointPosition[1] = y - 1;
				System.out.println("Waypoint set at "+_waypointPosition[0]+", "+_waypointPosition[1]);
				_ui.setStatus("waypoint set at "+_waypointPosition[0]+", "+_waypointPosition[1]);
			}
		}
	}

    /**
     * Function to reset the maze grids.
     *
     * @param mazeGrids
     */
    public void resetMaze(JButton[][] mazeGrids) {
        for (int x = 0; x < Arena.MAP_WIDTH; x++) {
            for (int y = 0; y < Arena.MAP_LENGTH; y++) {
                mazeGrids[x][y].setBackground(Color.BLACK);
                if ((x >= 0 & x <= 2 & y >= 12 & y <= 14) || (y >= 0 & y <= 2 & x >= 17 & x <= 19)) {
                    mazeGrids[x][y].setBackground(Color.PINK);
                }
            }
        }
    }

    /**
     * Function to overwrite and set the robot speed.
     * This is the text box in the middle panel.
     *
     * @param speed
     */
    public void setRobotSpeed(int speed) {
        _speed = speed;
        _ui.setStatus("robot speed set");
    }

    /**
     * Function to overwrite and set the robot orientation.
     * This is the text box in the middle panel.
     *
     * @param ori
     */
    public void setRobotOrientation(Orientation ori) {
        _robotOrientation = ori;
    }

    /**
     * Function to overwrite and set the robot coverage.
     * This is the text box in the middle panel.
     *
     * @param coverage
     */
    public void setCoverage(int coverage) {
        if (coverage > 100) {
            _ui.setStatus("warning: target coverage out of range");
        } else {
            _initialCoverage = coverage;
            _ui.setStatus("target coverage set");
        }
    }

    /**
     * Function to overwrite and set the time limit for exploration.
     * This is the text box in the middle panel.
     *
     * @param limit
     */
    public void setExploreTimeLimit(int limit) {
        _exploreTimeLimit = limit;
        _ui.setStatus("time limit for exploring set");
    }

    /**
     * Function to overwrite and set the time limit for fastest path.
     * This is the text box in the middle panel.
     *
     * @param limit
     */
    public void setFFPTimeLimit(int limit) {
        _ffpTimeLimit = limit;
        _ui.setStatus("time limit for fastest path set");
    }

    /**
     * Function to command the simulator robot to stop exploring and return to starting point.
     */
    public void stopExploring() {
        _hasReachedTimeThreshold = true;
        _exploreTimer.stop();
        _ui.setTimerMessage("Stopping exploration");
    }

    /**
     * This class handles exploration logic of the maze.
     */
    class ExploreTimeClass implements ActionListener {
		int _counter; 
		
		public ExploreTimeClass(int timeLimit) {
			_counter = timeLimit;
		}
		
		@Override
		public void actionPerformed(ActionEvent e) {
			
			_counter--;
			_ui.setTimer(_counter);
			
			if (_counter >= 0) {
				SwingWorker<Void, Float> getThreshold = new SwingWorker<Void, Float>() {
//					Path _backPath;
					@Override
					protected Void doInBackground() throws Exception {
						//float threshold;
						MazeExplorer explorer = MazeExplorer.getInstance();
						AStarPathFinder pathFinder = AStarPathFinder.getInstance();
						
//						_backPath = pathFinder.findFastestPath(_robotPosition[0], _robotPosition[1], 
//								MazeExplorer.START[0], MazeExplorer.START[1], explorer.getMazeRef());
//						System.out.println("Find fastest path back every turn");
//						_fastestBackPath = pathFinder.findFastestPath(_robotPosition[0], _robotPosition[1], 
//								MazeExplorer.START[0], MazeExplorer.START[1], explorer.getMazeRef());
						
						//threshold = _backPath.getNumOfSteps() * (1 / (float)_speed) + THRESHOLD_BUFFER_TIME; //If need buffer.
						publish((float)_counter);
						return null;
					}
					@Override
					protected void process(List<Float> chunks) {
						Float curThreshold = chunks.get(chunks.size() - 1);
						
						if (curThreshold == 0) {
							_hasReachedTimeThreshold = true;
						}
						
//						if (_counter <= curThreshold) {
//							_hasReachedTimeThreshold = true;
//						}
					}
				};
				
				if (_counter == 0) {
					_exploreTimer.stop();
					_ui.setTimerMessage("exploration: time out");
					Toolkit.getDefaultToolkit().beep();
				}
				
				getThreshold.execute();
			
			} 
		}
	}

    /**
     * Function handles exploration of the maze.
     */
    public void exploreMaze() {
    	
        if (!RobotSystem.isRealRun()) {
            if (!_ui.isIntExploreInput()) {
                _ui.setStatus("invalid input for exploration");
                _ui.setExploreBtnEnabled(true);
                return;
            }
            _ui.refreshExploreInput();
            Arena arena = Arena.getInstance();
            if (arena.getLayout() == null) {
                _ui.setStatus("warning: no layout loaded yet");
                return;
            }
        }
        
        MazeExplorer explorer = MazeExplorer.getInstance();
        Robot robot = Robot.getInstance();

        if (RobotSystem.isRealRun()) {
            _exploreTimeLimit = EXPLORE_TIME_LIMIT;
            _speed = EXPLORE_REAL_RUN_SPEED;
        }

        ExploreTimeClass timeActionListener = new ExploreTimeClass(_exploreTimeLimit);

        SwingWorker<Void, Void> exploreMaze = new SwingWorker<Void, Void>() {
            @Override
            protected Void doInBackground() throws Exception {

                if (!RobotSystem.isRealRun()) {
                    robot.setSpeed(_speed);
                }
                _hasReachedStart = false;
                
                explorer.explore(_robotPosition, _robotOrientation);
                
              
                //Compute the fastest path right after exploration for real run.
                if (RobotSystem.isRealRun()) {
                    AStarPathFinder pathFinder = AStarPathFinder.getInstance();
                    _fastestPath = pathFinder.findFastestPath(MazeExplorer.START[0], MazeExplorer.START[1],
                            MazeExplorer.GOAL[0], MazeExplorer.GOAL[1], explorer.getMazeRef());
                    Orientation bestOri = pathFinder.getBestInitialOrientation(_fastestPath);
                    explorer.adjustOrientationTo(bestOri);
                }

                return null;
            }

            @Override
            public void done() {

                _hasReachedStart = true;

                //Generate P1 & P2 map descriptors.
                String P1Descriptor, P2Descriptor;
                P1Descriptor = explorer.getP1Descriptor();
                P2Descriptor = explorer.getP2Descriptor();
                System.out.println("P1 descriptor: " + P1Descriptor);
                System.out.println("P2 descriptor: " + P2Descriptor);

                //Set simulator console status.
                _ui.setCoverageUpdate("actual coverage (%): " + String.format("%.1f", _actualCoverage));
                if (!_ui.getTimerMessage().equals("exploration: time out")) {
                    _ui.setTimerMessage("explored within time limit");
                }
                if (_exploreTimer.isRunning()) {
                    _exploreTimer.stop();
                }
                if (explorer.hasExploredTillGoal()) {
                    _ui.setStatus("exploration reaches goal zone");
                    if (!RobotSystem.isRealRun()) {
                        _ui.setFfpBtnEnabled(true);
                    }
                } else {
                    _ui.setStatus("exploration not reaches goal zone");
                }


                //Waiting for fastest path command for real run.
                if (!RobotSystem.isRealRun()) {
                    _ui.setExploreBtnEnabled(true);
                } else {
                    try {
                        String msgFFP = _pcClient.readMessage();
                        while (!msgFFP.equals(Message.START_FASTEST)) {
                            msgFFP = _pcClient.readMessage();
                        }
                        _ui.setStatus("Start finding fastest path");
                        findFastestPath(true);
                        //findFastestPath(false);
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
            }
        };

        SwingWorker<Void, Void> updateCoverage = new SwingWorker<Void, Void>() {
            @Override
            protected Void doInBackground() throws Exception {
                int numExplored;
                JButton[][] mazeGrids = _ui.getMazeGrids();
                while (!_hasReachedStart) {
                    numExplored = 0;
                    for (int x = 0; x < Arena.MAP_WIDTH; x++) {
                        for (int y = 0; y < Arena.MAP_LENGTH; y++) {
                            if (mazeGrids[x][y].getBackground() != Color.BLACK) {
                                numExplored++;
                            }
                        }
                    }
                    _actualCoverage = (float) (100 * numExplored) / (float) (Arena.MAP_LENGTH * Arena.MAP_WIDTH);
                    _ui.setCoverageUpdate(_actualCoverage);

                    //if (_initialCoverage <= _actualCoverage)
                    //    stopExploring();
                }
                return null;
            }
        };


        _exploreTimer = new Timer(1000, timeActionListener);
        _exploreTimer.start();
        _ui.setStatus("robot exploring");
        _hasReachedTimeThreshold = false;
        exploreMaze.execute();
        updateCoverage.execute();

    }

    /**
     * This class handles Fastest Path logic of the maze.
     */
    class FastestPathTimeClass implements ActionListener {
        int _timeLimit;
        int _counter;

        public FastestPathTimeClass(int timeLimit) {
            _timeLimit = timeLimit;
            _counter = 0;
        }

        @Override
        public void actionPerformed(ActionEvent e) {

            _timeLimit--;
            _counter++;
            _ui.setCoverageUpdate("Time passed (sec): " + _counter);
            _ui.setTimer(_timeLimit);
            if (_timeLimit == 0) {
                _ffpTimer.stop();
                _ui.setCoverageUpdate("");
                _ui.setTimerMessage("fastest path: time out");
                Toolkit.getDefaultToolkit().beep();
            }
        }
    }

    public void findFastestPath(boolean s) {
    	
    	 if (!RobotSystem.isRealRun()) {
	            if (!_ui.isIntFFPInput()) {
	                _ui.setStatus("invalid input for finding fastest path");
	                _ui.setFfpBtnEnabled(true);
	                return;
	            }

	            _ui.refreshFfpInput();
	        }
    	 
    	 //A *
    	if(s == true) {
    		SwingWorker<Void, Void> findFastestPath = new SwingWorker<Void, Void>() {
	            @Override
	            protected Void doInBackground() throws Exception {

	                MazeExplorer explorer = MazeExplorer.getInstance();
	                AStarPathFinder pathFinder = AStarPathFinder.getInstance();

	                if (!RobotSystem.isRealRun()) {
	                    _fastestPath = pathFinder.findFastestPath(MazeExplorer.START[0], MazeExplorer.START[1],
	                            MazeExplorer.GOAL[0], MazeExplorer.GOAL[1], explorer.getMazeRef());
	                }

	                pathFinder.moveRobotAlongFastestPath(_fastestPath, explorer.getRobotOrientation());

	                ArrayList<Path.Step> steps = _fastestPath.getSteps();
	                JButton[][] mazeGrids = _ui.getMazeGrids();
	                for (Path.Step step : steps) {
	                    int x = step.getX();
	                    int y = step.getY();
	                    mazeGrids[19 - y][x].setBackground(Color.MAGENTA);
	                }
	                return null;
	            }

	            @Override
	            public void done() {
	                _ui.setStatus("Fastest path found");

	                if (!_ui.getTimerMessage().equals("Fastest path: time out")) {
	                    _ui.setTimerMessage("Fastest path: within time limit");
	                }
	                if (_ffpTimer.isRunning()) {
	                    _ffpTimer.stop();
	                }

	            }
	        };


	        if (RobotSystem.isRealRun()) {
	            _ffpTimeLimit = FFP_TIME_LIMIT;
	        }
	        FastestPathTimeClass timeActionListener = new FastestPathTimeClass(_ffpTimeLimit);
	        _ffpTimer = new Timer(1000, timeActionListener);
	        _ffpTimer.start();
	        _ui.setStatus("Robot finding fastest path");
	        findFastestPath.execute();
    	
    	}
    	else {
    		// MANUALLY SET WAY POINTS
    		
    		SwingWorker<Void, Void> findFastestPath = new SwingWorker<Void, Void>() {
    			@Override
    			protected Void doInBackground() throws Exception {
    				MazeExplorer explorer = MazeExplorer.getInstance();
    				AStarPathFinder pathFinder = AStarPathFinder.getInstance();
    	
    				if(!RobotSystem.isRealRun()) {
    					_fastestPath = pathFinder.findFastestPath(MazeExplorer.START[0], MazeExplorer.START[1], 
    							_waypointPosition[0], _waypointPosition[1], explorer.getMazeRef());
    					_fastestPath2 = pathFinder.findFastestPath(_waypointPosition[0], _waypointPosition[1], 
    							MazeExplorer.GOAL[0], MazeExplorer.GOAL[1], explorer.getMazeRef());
    	
    					_fastestPath.combineSteps(_fastestPath2.getSteps());
    				}
    				System.out.println("Fastest Path: Start");
    				pathFinder.moveRobotAlongFastestPath(_fastestPath, explorer.getRobotOrientation());
    				System.out.println("Fastest Path: End");
    				ArrayList<Path.Step> steps = _fastestPath.getSteps();
    				JButton[][] mazeGrids = _ui.getMazeGrids();
    				for (Path.Step step : steps) {
    					int x = step.getX();
    					int y = step.getY();
    					mazeGrids[19-y][x].setBackground(Color.YELLOW);
    				}
    				
    				return null;
    			}
    			@Override
    			public void done() {
    				Robot robot = Robot.getInstance();
    				_robotOrientation = robot.calibrateAtEndZone(_robotOrientation);
    				PCClient pcClient = PCClient.getInstance();
    				if (RobotSystem.isRealRun()) {
    					pcClient.sendMessageToAndroid(Message.EXPLORE_DONE);
    				}
    				_ui.setStatus("fastest path found");
    				if (!_ui.getTimerMessage().equals("fastest path: time out")) {
    					_ui.setTimerMessage("fastest path: within time limit");
    				}
    				if (_ffpTimer.isRunning()) {
    					_ffpTimer.stop(); 
    				}
    			}
    		};
    		
    		if (RobotSystem.isRealRun()) {
    			_ffpTimeLimit = FFP_TIME_LIMIT;
    		}
    		FastestPathTimeClass timeActionListener = new FastestPathTimeClass(_ffpTimeLimit);
    		_ffpTimer = new Timer(1000, timeActionListener);
    		_ffpTimer.start();
    		_ui.setStatus("robot finding fastest path");
    		findFastestPath.execute();
    	}
       
    }

    public void moveRobotForward() {
        JButton[][] mazeGrids = _ui.getMazeGrids();

        switch (_robotOrientation) {
            case NORTH:
                for (int i = 17 - _robotPosition[1]; i <= 19 - _robotPosition[1]; i++) {
                    for (int j = _robotPosition[0] - 1; j <= _robotPosition[0] + 1; j++) {
                        if (i == 17 - _robotPosition[1] && j == _robotPosition[0]) {
                            mazeGrids[i][j].setBackground(Color.BLUE);
                        } else {
                            mazeGrids[i][j].setBackground(Color.GREEN);
                        }
                    }
                }
                _robotPosition[1] = _robotPosition[1] + 1;
                break;
            case SOUTH:
                for (int i = 19 - _robotPosition[1]; i <= 21 - _robotPosition[1]; i++) {
                    for (int j = _robotPosition[0] - 1; j <= _robotPosition[0] + 1; j++) {
                        if (i == 21 - _robotPosition[1] && j == _robotPosition[0]) {
                            mazeGrids[i][j].setBackground(Color.BLUE);
                        } else {
                            mazeGrids[i][j].setBackground(Color.GREEN);
                        }
                    }
                }
                _robotPosition[1] = _robotPosition[1] - 1;
                break;
            case EAST:
                for (int i = 18 - _robotPosition[1]; i <= 20 - _robotPosition[1]; i++) {
                    for (int j = _robotPosition[0]; j <= _robotPosition[0] + 2; j++) {
                        if (i == 19 - _robotPosition[1] && j == _robotPosition[0] + 2) {
                            mazeGrids[i][j].setBackground(Color.BLUE);
                        } else {
                            mazeGrids[i][j].setBackground(Color.GREEN);
                        }
                    }
                }
                _robotPosition[0] = _robotPosition[0] + 1;
                break;
            case WEST:
                for (int i = 18 - _robotPosition[1]; i <= 20 - _robotPosition[1]; i++) {
                    for (int j = _robotPosition[0] - 2; j <= _robotPosition[0]; j++) {
                        if (i == 19 - _robotPosition[1] && j == _robotPosition[0] - 2) {
                            mazeGrids[i][j].setBackground(Color.BLUE);
                        } else {
                            mazeGrids[i][j].setBackground(Color.GREEN);
                        }
                    }
                }
                _robotPosition[0] = _robotPosition[0] - 1;
        }
        updateMazeColor();
    }

    public void turnRobotRight() {
        JButton[][] mazeGrids = _ui.getMazeGrids();

        switch (_robotOrientation) {
            case NORTH:
                mazeGrids[18 - _robotPosition[1]][_robotPosition[0]].setBackground(Color.GREEN);
                mazeGrids[19 - _robotPosition[1]][_robotPosition[0] + 1].setBackground(Color.BLUE);
                _robotOrientation = Orientation.EAST;
                break;
            case SOUTH:
                mazeGrids[20 - _robotPosition[1]][_robotPosition[0]].setBackground(Color.GREEN);
                mazeGrids[19 - _robotPosition[1]][_robotPosition[0] - 1].setBackground(Color.BLUE);
                _robotOrientation = Orientation.WEST;
                break;
            case EAST:
                mazeGrids[19 - _robotPosition[1]][_robotPosition[0] + 1].setBackground(Color.GREEN);
                mazeGrids[20 - _robotPosition[1]][_robotPosition[0]].setBackground(Color.BLUE);
                _robotOrientation = Orientation.SOUTH;
                break;
            case WEST:
                mazeGrids[19 - _robotPosition[1]][_robotPosition[0] - 1].setBackground(Color.GREEN);
                mazeGrids[18 - _robotPosition[1]][_robotPosition[0]].setBackground(Color.BLUE);
                _robotOrientation = Orientation.NORTH;
        }
        updateMazeColor();
    }

    public void turnRobotLeft() {
        JButton[][] mazeGrids = _ui.getMazeGrids();

        switch (_robotOrientation) {
            case NORTH:
                mazeGrids[18 - _robotPosition[1]][_robotPosition[0]].setBackground(Color.GREEN);
                mazeGrids[19 - _robotPosition[1]][_robotPosition[0] - 1].setBackground(Color.BLUE);
                _robotOrientation = Orientation.WEST;
                break;
            case SOUTH:
                mazeGrids[20 - _robotPosition[1]][_robotPosition[0]].setBackground(Color.GREEN);
                mazeGrids[19 - _robotPosition[1]][_robotPosition[0] + 1].setBackground(Color.BLUE);
                _robotOrientation = Orientation.EAST;
                break;
            case EAST:
                mazeGrids[19 - _robotPosition[1]][_robotPosition[0] + 1].setBackground(Color.GREEN);
                mazeGrids[18 - _robotPosition[1]][_robotPosition[0]].setBackground(Color.BLUE);
                _robotOrientation = Orientation.NORTH;
                break;
            case WEST:
                mazeGrids[19 - _robotPosition[1]][_robotPosition[0] - 1].setBackground(Color.GREEN);
                mazeGrids[20 - _robotPosition[1]][_robotPosition[0]].setBackground(Color.BLUE);
                _robotOrientation = Orientation.SOUTH;
        }
        updateMazeColor();
    }

    public void updateMazeColor() {
        JButton[][] mazeGrids = _ui.getMazeGrids();
        MazeExplorer explorer = MazeExplorer.getInstance();
        int[][] mazeRef = explorer.getMazeRef();
        for (int i = 0; i < Arena.MAP_LENGTH; i++) {
            for (int j = 0; j < Arena.MAP_WIDTH; j++) {
                if (mazeRef[i][j] == MazeExplorer.IS_EMPTY) {
                    if (i < _robotPosition[0] - 1 || i > _robotPosition[0] + 1 ||
                            j < _robotPosition[1] - 1 || j > _robotPosition[1] + 1) {
                        if ((i >= MazeExplorer.START[0] - 1 && i <= MazeExplorer.START[0] + 1
                                && j >= MazeExplorer.START[1] - 1 && j <= MazeExplorer.START[1] + 1)
                                || (i >= MazeExplorer.GOAL[0] - 1 && i <= MazeExplorer.GOAL[0] + 1
                                && j >= MazeExplorer.GOAL[1] - 1 && j <= MazeExplorer.GOAL[1] + 1)) {
                            mazeGrids[19 - j][i].setBackground(Color.PINK);
                        } else {
                            mazeGrids[19 - j][i].setBackground(Color.WHITE);
                        }
                    }
                } else if (mazeRef[i][j] == MazeExplorer.IS_OBSTACLE) {
                    mazeGrids[19 - j][i].setBackground(Color.RED);
                }
            }
        }
    }

}
