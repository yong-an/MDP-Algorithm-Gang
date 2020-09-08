package simulator;

import java.awt.BorderLayout;
import java.awt.CardLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.IOException;
import java.nio.file.FileSystems;
import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.border.CompoundBorder;
import javax.swing.border.EmptyBorder;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;
import javax.swing.text.BadLocationException;
import javax.swing.text.Document;

import main.RobotSystem;
import simulator.arena.Arena;
import simulator.arena.FileReaderWriter;

import java.awt.Font;
import javax.swing.JTextField;
import java.awt.FlowLayout;

/**
 * This Java file handles the simulator UI settings.
 */
public class UI extends JFrame implements ActionListener {

    private static final String EXPLORE_PANEL = "Explore Arena";
    private static final String FFP_PANEL = "Fastest Path";
    private static final long serialVersionUID = 1L;
    private JPanel _contentPane, _mapPane, _ctrlPane, _mazePane;
    private JLabel _status, _timer, _coverageUpdate;
    private JButton[][] _mapGrids, _mazeGrids;
    private Controller _controller;
    private JTextField[] _exploreTextFields, _ffpTextFields;
    private JButton _exploreBtn, _ffpBtn, _stopExploreBtn, _loadBtn;

    /**
     * Creation of the simulator UI.
     */
    public UI() {
        super("MDP Simulator - GRP23 - Exploration & Fastest Path");
        setResizable(false);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        _contentPane = new JPanel();
        _contentPane.setBorder(new EmptyBorder(5, 5, 5, 5));
        _contentPane.setLayout(new BorderLayout(0, 0));
        setContentPane(_contentPane);
        initContentPane(_contentPane);
        pack();
    }

    /**
     * Function to enable / disable the explore button.
     *
     * @param value
     */
    public void setExploreBtnEnabled(boolean value) {
        _exploreBtn.setEnabled(value);
    }

    /**
     * Function to enable / disable the fastest path button.
     *
     * @param value
     */
    public void setFfpBtnEnabled(boolean value) {
        _ffpBtn.setEnabled(value);
    }

    /**
     * Function to return the UI content pane.
     *
     * @return
     */
    public JPanel getContentPane() {
        return _contentPane;
    }

    /**
     * Function to return the maze grids.
     *
     * @return
     */
    public JButton[][] getMazeGrids() {
        return _mazeGrids;
    }

    /**
     * Function to set the maze grids.
     *
     * @param mazeGrids
     */
    public void setMazeGrids(JButton[][] mazeGrids) {
        _mazeGrids = mazeGrids;
    }

    /**
     * Function to create the left, middle, right panels.
     * Starting with the left panel.
     * Which includes a plottable reference map.
     * 2 Control buttons [load] & [clear] map.
     *
     * @param contentPane
     */
    private void initContentPane(JPanel contentPane) {
        _mapPane = new JPanel(new FlowLayout());
        _mapPane.setPreferredSize(new Dimension(450, 650));
        JPanel map = new JPanel();
        map.setLayout(new GridLayout(Arena.MAP_WIDTH, Arena.MAP_LENGTH));
        map.setPreferredSize(new Dimension(450, 600));
        _mapGrids = new JButton[Arena.MAP_WIDTH][Arena.MAP_LENGTH];
        for (int x = 0; x < Arena.MAP_WIDTH; x++) {
            for (int y = 0; y < Arena.MAP_LENGTH; y++) {
                _mapGrids[x][y] = new JButton();

                if (RobotSystem.isRealRun()) {
                    _mapGrids[x][y].setEnabled(false);
                    _mapGrids[x][y].setBackground(Color.GRAY);
                } else {
                    _mapGrids[x][y].setActionCommand("ToggleObstacleAt " + x + "," + y);
                    _mapGrids[x][y].addActionListener(this);
                    _mapGrids[x][y].setBorder(BorderFactory.createLineBorder(Color.GRAY));
                    _mapGrids[x][y].setBackground(Color.WHITE);
                    if ((x >= 0 & x <= 2 & y >= 12 & y <= 14) || (y >= 0 & y <= 2 & x >= 17 & x <= 19)) {
                        _mapGrids[x][y].setEnabled(false);
                        _mapGrids[x][y].setBackground(Color.CYAN);
                        if (x == 1 & y == 13) {
                            _mapGrids[x][y].setText("E");
                        } else if (x == 18 && y == 1) {
                            _mapGrids[x][y].setText("S");
                        }
                    }
                }

                map.add(_mapGrids[x][y]);
            }
        }

        if (!RobotSystem.isRealRun()) {
            loadArenaFromDisk();
        }

        _mapPane.add(map);
        _loadBtn = new JButton("Load Map ==>");
        JButton clearMap = new JButton("Clear Obstacles");

        if (RobotSystem.isRealRun()) {
            _loadBtn.setEnabled(false);
            clearMap.setEnabled(false);
        } else {
            _loadBtn.setActionCommand("LoadMap");
            _loadBtn.addActionListener(this);
            // disable button when first loaded
            _loadBtn.setEnabled(false);

            clearMap.setActionCommand("ClearMap");
            clearMap.addActionListener(this);
        }
        _mapPane.add(_loadBtn);
        _mapPane.add(clearMap);
        contentPane.add(_mapPane, BorderLayout.WEST);

        //==============================================================================================
        //Followed by the middle panel.
        //Which includes a drop down list to change the settings to exploration / fastest path.
        //A few text boxes to change the simulator settings.
        //And a current status panel.

        //Combo box drop down list [Exploration / Fastest Path].
        _ctrlPane = new JPanel(new BorderLayout());
        _ctrlPane.setBorder(new EmptyBorder(50, 20, 50, 20));
        String comboBoxItems[] = {EXPLORE_PANEL, FFP_PANEL};
        JComboBox cbCtrlSwitch = new JComboBox(comboBoxItems);
        cbCtrlSwitch.setFont(new Font("Tahoma", Font.BOLD, 16));
        cbCtrlSwitch.setEditable(false);
        cbCtrlSwitch.addActionListener(this);
        cbCtrlSwitch.setActionCommand("SwitchCtrl");
        _ctrlPane.add(cbCtrlSwitch, BorderLayout.NORTH);

        //Controls for the Exploration.
        JLabel[] exploreCtrlLabels = new JLabel[4];
        _exploreTextFields = new JTextField[4];
        _exploreBtn = new JButton("Explore");
        _stopExploreBtn = new JButton("Stop Exploration");
        _stopExploreBtn.setVisible(false);
        _stopExploreBtn.setEnabled(false);

        if (RobotSystem.isRealRun()) {
            _exploreBtn.setEnabled(false);
        } else {
            _exploreBtn.setActionCommand("ExploreMaze");
            _exploreBtn.addActionListener(this);

            _stopExploreBtn.setActionCommand("stopExplore");
            _stopExploreBtn.addActionListener(this);
        }

        exploreCtrlLabels[0] = new JLabel("Robot initial position: ");
        exploreCtrlLabels[1] = new JLabel("Speed (steps/sec): ");
        exploreCtrlLabels[2] = new JLabel("Target coverage (%): ");
        exploreCtrlLabels[3] = new JLabel("Time limit (sec): ");
        exploreCtrlLabels[0].setVisible(false);
        exploreCtrlLabels[1].setVisible(false);
        exploreCtrlLabels[2].setVisible(false);
        exploreCtrlLabels[3].setVisible(false);
        for (int i = 0; i < 4; i++) {
            _exploreTextFields[i] = new JTextField(10);
            _exploreTextFields[i].setVisible(false);
            if (RobotSystem.isRealRun()) {
                _exploreTextFields[i].setEditable(false);
            }
        }

        JPanel exploreInputPane = new JPanel(new GridLayout(4, 2));

        exploreInputPane.add(exploreCtrlLabels[0]);
        exploreInputPane.add(_exploreTextFields[0]);
        exploreInputPane.add(exploreCtrlLabels[1]);
        exploreInputPane.add(_exploreTextFields[1]);
        exploreInputPane.add(exploreCtrlLabels[2]);
        exploreInputPane.add(_exploreTextFields[2]);
        exploreInputPane.add(exploreCtrlLabels[3]);
        exploreInputPane.add(_exploreTextFields[3]);

        if (!RobotSystem.isRealRun()) {
            _exploreTextFields[0].setEditable(false);
            exploreCtrlLabels[0].setFont(new Font("Tahoma", Font.PLAIN, 14));
            _exploreTextFields[0].setText("2,2");
            _exploreTextFields[0].setFont(new Font("Tahoma", Font.PLAIN, 14));
            _exploreTextFields[0].getDocument().addDocumentListener(new InitialPositionListener());
            _exploreTextFields[0].getDocument().putProperty("name", "Robot Initial Position");

            exploreCtrlLabels[1].setFont(new Font("Tahoma", Font.PLAIN, 14));
            _exploreTextFields[1].setText("10");
            _exploreTextFields[1].setFont(new Font("Tahoma", Font.PLAIN, 14));
            _exploreTextFields[1].getDocument().addDocumentListener(new InitialPositionListener());
            _exploreTextFields[1].getDocument().putProperty("name", "Robot Explore Speed");

            exploreCtrlLabels[2].setFont(new Font("Tahoma", Font.PLAIN, 14));
            _exploreTextFields[2].setText("100");
            _exploreTextFields[2].setFont(new Font("Tahoma", Font.PLAIN, 14));
            _exploreTextFields[2].getDocument().addDocumentListener(new InitialPositionListener());
            _exploreTextFields[2].getDocument().putProperty("name", "Target Coverage");

            exploreCtrlLabels[3].setFont(new Font("Tahoma", Font.PLAIN, 14));
            _exploreTextFields[3].setText("360");
            _exploreTextFields[3].setFont(new Font("Tahoma", Font.PLAIN, 14));
            _exploreTextFields[3].getDocument().addDocumentListener(new InitialPositionListener());
            _exploreTextFields[3].getDocument().putProperty("name", "Robot Explore Time Limit");
        }

        JPanel exploreBtnPane = new JPanel();
        exploreBtnPane.add(_exploreBtn);
        exploreBtnPane.add(_stopExploreBtn);

        JPanel exploreCtrlPane = new JPanel();
        exploreCtrlPane.add(exploreInputPane);
        exploreCtrlPane.add(exploreBtnPane);
        exploreCtrlPane.setBorder(new EmptyBorder(20, 20, 20, 20));

        //Controls for the Fastest Path
        JLabel[] ffpCtrlLabels = new JLabel[2];
        _ffpTextFields = new JTextField[2];
        _ffpBtn = new JButton("Navigate");

        if (RobotSystem.isRealRun()) {
            _ffpBtn.setEnabled(false);
        } else {
            _ffpBtn.setActionCommand("FindFastestPath");
            _ffpBtn.addActionListener(this);
            _ffpBtn.setEnabled(false);
        }

        ffpCtrlLabels[0] = new JLabel("Speed (steps/sec): ");
        ffpCtrlLabels[1] = new JLabel("Time limit (sec): ");
        ffpCtrlLabels[0].setVisible(false);
        ffpCtrlLabels[1].setVisible(false);
        for (int i = 0; i < 2; i++) {
            _ffpTextFields[i] = new JTextField(10);
            _ffpTextFields[i].setVisible(false);
            if (RobotSystem.isRealRun()) {
                _ffpTextFields[i].setEditable(false);
            }
        }

        JPanel ffpInputPane = new JPanel(new GridLayout(2, 2));
        ffpInputPane.add(ffpCtrlLabels[0]);
        ffpInputPane.add(_ffpTextFields[0]);
        ffpInputPane.add(ffpCtrlLabels[1]);
        ffpInputPane.add(_ffpTextFields[1]);

        if (!RobotSystem.isRealRun()) {
            ffpCtrlLabels[0].setFont(new Font("Tahoma", Font.PLAIN, 14));
            _ffpTextFields[0].setFont(new Font("Tahoma", Font.PLAIN, 14));
            _ffpTextFields[0].setEditable(false);

            _ffpTextFields[1].setText("120");
            ffpCtrlLabels[1].setFont(new Font("Tahoma", Font.PLAIN, 14));
            _ffpTextFields[1].setFont(new Font("Tahoma", Font.PLAIN, 14));
            _ffpTextFields[1].getDocument().addDocumentListener(new InitialPositionListener());
            _ffpTextFields[1].getDocument().putProperty("name", "Robot FFP Time Limit");
        }

        JPanel ffpBtnPane = new JPanel();
        ffpBtnPane.add(_ffpBtn);

        JPanel ffpCtrlPane = new JPanel();
        ffpCtrlPane.add(ffpInputPane);
        ffpCtrlPane.add(ffpBtnPane);
        ffpCtrlPane.setBorder(new EmptyBorder(20, 20, 20, 20));

        //A card panel to swap the controls panels based on what u select on the combo box.
        JPanel cardPane = new JPanel(new CardLayout());
        cardPane.add(exploreCtrlPane, EXPLORE_PANEL);
        cardPane.add(ffpCtrlPane, FFP_PANEL);
        cardPane.setPreferredSize(new Dimension(280, 300));
        _ctrlPane.add(cardPane, BorderLayout.CENTER);

        //Create the status panel.
        JPanel statusPane = new JPanel(new BorderLayout());
        JLabel statusLabel = new JLabel("Status Console:");
        statusPane.add(statusLabel, BorderLayout.NORTH);
        JPanel statusConsole = new JPanel(new GridLayout(3, 1));
        statusConsole.setBackground(Color.LIGHT_GRAY);
        statusConsole.setPreferredSize(new Dimension(280, 100));
        _status = new JLabel("waiting for commands...");
        _status.setHorizontalAlignment(JLabel.CENTER);
        _timer = new JLabel();
        _timer.setHorizontalAlignment(JLabel.CENTER);
        _coverageUpdate = new JLabel();
        _coverageUpdate.setHorizontalAlignment(JLabel.CENTER);
        statusConsole.add(_status);
        statusConsole.add(_coverageUpdate);
        statusConsole.add(_timer);
        statusPane.add(statusConsole, BorderLayout.CENTER);
        _ctrlPane.add(statusPane, BorderLayout.SOUTH);
        contentPane.add(_ctrlPane, BorderLayout.CENTER);
        statusPane.setVisible(false);

        //==============================================================================================
        //Lastly the right panel.
        //Which displays the simulated run.

        _mazePane = new JPanel(new FlowLayout());
        _mazePane.setPreferredSize(new Dimension(450, 650));
        JPanel maze = new JPanel();
        maze.setLayout(new GridLayout(Arena.MAP_WIDTH, Arena.MAP_LENGTH));
        maze.setPreferredSize(new Dimension(450, 600));
        _mazeGrids = new JButton[Arena.MAP_WIDTH][Arena.MAP_LENGTH];
        for (int x = 0; x < Arena.MAP_WIDTH; x++) {
            for (int y = 0; y < Arena.MAP_LENGTH; y++) {
                _mazeGrids[x][y] = new JButton();
                _mazeGrids[x][y].setEnabled(false);
                _mazeGrids[x][y].setBorder(BorderFactory.createLineBorder(Color.GRAY));
                if (x == 10) {
                    _mazeGrids[x][y]
                            .setBorder(new CompoundBorder(BorderFactory.createMatteBorder(3, 0, 0, 0, Color.BLUE),
                                    BorderFactory.createMatteBorder(0, 1, 1, 1, Color.GRAY)));
                }
                _mazeGrids[x][y].setBackground(Color.BLACK);
                maze.add(_mazeGrids[x][y]);
                if ((x >= 0 & x <= 2 & y >= 12 & y <= 14) || (y >= 0 & y <= 2 & x >= 17 & x <= 19)) {
                    _mazeGrids[x][y].setEnabled(false);
                    _mazeGrids[x][y].setBackground(Color.PINK);
                    if (x == 1 & y == 13) {
                        _mazeGrids[x][y].setText("G");
                    } else if (x == 18 && y == 1) {
                        _mazeGrids[x][y].setText("S");
                    }
                }
            }
        }
        _mazePane.add(maze);
        contentPane.add(_mazePane, BorderLayout.EAST);
    }

    /**
     * This function handles the buttons listener on the entire UI.
     *
     * @param e
     */
    @Override
    public void actionPerformed(ActionEvent e) {
        String cmd = e.getActionCommand();
        _controller = Controller.getInstance();
        if (cmd.matches("ToggleObstacleAt [0-9]+,[0-9]+")) {
            int index = cmd.indexOf(",");
            int x = Integer.parseInt(cmd.substring(17, index));
            int y = Integer.parseInt(cmd.substring(index + 1));
            _controller.toggleObstacle(_mapGrids, x, y);
            // enable load map button when map layout change
            _loadBtn.setEnabled(true);
            System.out.println("Toggle at " + x + ", " + y);
        } else if (cmd.equals("SwitchCtrl")) {
            JComboBox cb = (JComboBox) e.getSource();
            JPanel cardPanel = (JPanel) _ctrlPane.getComponent(1);
            _controller.switchComboBox(cb, cardPanel);
        } else if (cmd.equals("LoadMap")) {
            _controller.loadMap(_mapGrids);
            // disable load map button when map layout has been loaded
            _loadBtn.setEnabled(false);
        } else if (cmd.equals("ClearMap")) {
            _controller.clearMap(_mapGrids);
        } else if (cmd.equals("ExploreMaze")) {
            _exploreBtn.setEnabled(false);
            _stopExploreBtn.setEnabled(true);
            _controller.exploreMaze();
        } else if (cmd.equals("FindFastestPath")) {
            _ffpBtn.setEnabled(false);
            _controller.findFastestPath();
        } else if (cmd.equals("stopExplore")) {
            _stopExploreBtn.setEnabled(false);
            _controller.stopExploring();

        }
    }

    /**
     * This function allows you to set the status message.
     *
     * @param message
     */
    public void setStatus(String message) {
        _status.setText(message);
    }

    /**
     * This function allows you to set status timer.
     *
     * @param timeLeft
     */
    public void setTimer(int timeLeft) {
        _timer.setText("Time left (sec): " + timeLeft);
    }

    /**
     * This function will return you the status timer message value.
     *
     * @return
     */
    public String getTimerMessage() {
        return _timer.getText();
    }

    /**
     * This function allows you to set the status timer message.
     *
     * @param message
     */
    public void setTimerMessage(String message) {
        _timer.setText(message);
    }

    /**
     * This function calculates the overall explored tiles left by returning a % value.
     *
     * @param coverage
     */
    public void setCoverageUpdate(Float coverage) {
        _coverageUpdate.setText("Coverage (%): " + String.format("%.1f", coverage));
    }

    /**
     * This function allows the setting of coverage status.
     *
     * @param message
     */
    public void setCoverageUpdate(String message) {
        _coverageUpdate.setText(message);
    }

    //==============================================================================================

    /**
     * This document listener class will dynamically update the simulator robot position.
     * As it moves [Live update].
     */
    class InitialPositionListener implements DocumentListener {
        public void insertUpdate(DocumentEvent e) {
            update(e);
        }

        public void removeUpdate(DocumentEvent e) {
            update(e);
        }

        public void changedUpdate(DocumentEvent e) {

        }

        private void update(DocumentEvent e) {
            _controller = Controller.getInstance();
            Document doc = (Document) e.getDocument();
            String name = (String) doc.getProperty("name");
            if (name.equals("Robot Initial Position")) {
                try {
                    String position = doc.getText(0, doc.getLength());
                    if (position.matches("[0-9]+,[0-9]+")) {
                        int index = position.indexOf(",");
                        int x = Integer.parseInt(position.substring(0, index));
                        int y = Integer.parseInt(position.substring(index + 1));
                        _controller.resetRobotInMaze(_mazeGrids, x, y);
                    } else {
                        _controller.resetMaze(_mazeGrids);
                        _status.setText("robot initial position not set");
                    }
                } catch (BadLocationException ex) {
                    ex.printStackTrace();
                }
            } else if (name.equals("Robot Explore Speed")) {
                try {
                    String speed = doc.getText(0, doc.getLength());
                    if (speed.matches("[0-9]+")) {
                        _controller.setRobotSpeed(Integer.parseInt(speed));
                        _ffpTextFields[0].setText(speed);
                    } else {
                        _status.setText("robot speed not set");
                    }
                } catch (BadLocationException ex) {
                    ex.printStackTrace();
                }
            } else if (name.equals("Target Coverage")) {
                try {
                    String coverage = doc.getText(0, doc.getLength());
                    if (coverage.matches("[0-9]+")) {
                        _controller.setCoverage(Integer.parseInt(coverage));
                        _coverageUpdate.setText("Coverage (%): 0");
                    } else {
                        _status.setText("target coverage not set");
                    }
                } catch (BadLocationException ex) {
                    ex.printStackTrace();
                }
            } else if (name.equals("Robot Explore Time Limit")) {
                try {
                    String timeLimit = doc.getText(0, doc.getLength());
                    if (timeLimit.matches("[0-9]+")) {
                        _controller.setExploreTimeLimit(Integer.parseInt(timeLimit));
                        _timer.setText("Time left (sec): " + timeLimit);
                    } else {
                        _status.setText("time limit for exploring not set");
                    }
                } catch (BadLocationException ex) {
                    ex.printStackTrace();
                }
            } else if (name.equals("Robot FFP Time Limit")) {
                try {
                    String timeLimit = doc.getText(0, doc.getLength());
                    if (timeLimit.matches("[0-9]+")) {
                        _controller.setFFPTimeLimit(Integer.parseInt(timeLimit));
                        _coverageUpdate.setText("");
                        _timer.setText("Time left (sec): " + timeLimit);
                    } else {
                        _status.setText("time limit for fastest path not set");
                    }
                } catch (BadLocationException ex) {
                    ex.printStackTrace();
                }
            }
        }

    }

    /**
     * After plotting the obstacles at the reference map.
     * Click load -> it will generate and save map-descriptor text file for future reference.
     * This function will then load the map-descriptor text file and set it as the current arena.
     * Take note: Arena.MAP_LENGTH * i returns each row.
     * While j gives the position in each row.
     * The _mapGrids array will be exactly the same as reference map UI.
     */
    private void loadArenaFromDisk() {
        FileReaderWriter fileReader;
        try {
            fileReader = new FileReaderWriter(FileSystems.getDefault().getPath(Controller.ARENA_DESCRIPTOR_PATH));
            String arenaDescriptor = fileReader.read();
            System.out.println(arenaDescriptor);
            if (!arenaDescriptor.equals("")) {
                for (int i = 0; i < Arena.MAP_WIDTH; i++) {
                    for (int j = 0; j < Arena.MAP_LENGTH; j++) {
                        if (arenaDescriptor.charAt(Arena.MAP_LENGTH * i + j) == '1') {
                            _mapGrids[19 - i][j].setBackground(Color.RED);
                        }
                    }
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
        }

        Arena arena = Arena.getInstance();
        arena.setLayout(_mapGrids);
    }

    /**
     * This function will refresh the explore input.
     */
    public void refreshExploreInput() {
        for (int i = 0; i < 4; i++) {
            _exploreTextFields[i].setText(_exploreTextFields[i].getText());
        }
    }

    /**
     * This function will refresh the fastest path input.
     */
    public void refreshFfpInput() {
        for (int i = 0; i < 2; i++) {
            _ffpTextFields[i].setText(_ffpTextFields[i].getText());
        }

    }

    /**
     * This function will verify if the exploration input is valid.
     * @return
     */
    public boolean isIntExploreInput() {
        String[] exploreInput = new String[4];
        for (int i = 0; i < 4; i++) {
            exploreInput[i] = _exploreTextFields[i].getText();
        }

        if (!exploreInput[0].matches("[0-9]+,[0-9]+")) {
            return false;
        } else {
            int posX, posY, index;
            index = exploreInput[0].indexOf(",");
            posX = Integer.parseInt(exploreInput[0].substring(0, index));
            posY = Integer.parseInt(exploreInput[0].substring(index + 1));
            if (posX > Arena.MAP_LENGTH || posY > Arena.MAP_WIDTH) {
                return false;
            }
        }

        if (!exploreInput[1].matches("[0-9]+") || !exploreInput[2].matches("[0-9]+") ||
                !exploreInput[3].matches("[0-9]+")) {
            return false;
        }

        if (exploreInput[2].matches("[0-9]+")) {
            int coverage;
            coverage = Integer.parseInt(exploreInput[2]);
            if (coverage > 100) {
                return false;
            }
        }

        return true;
    }

    /**
     * This function will verify if the fastest path input is valid.
     * @return
     */
    public boolean isIntFFPInput() {
        String[] ffpInput = new String[2];
        for (int i = 0; i < 2; i++) {
            ffpInput[i] = _ffpTextFields[i].getText();
        }

        if (!ffpInput[0].matches("[0-9]+") || !ffpInput[1].matches("[0-9]+")) {
            return false;
        }

        return true;
    }

}
