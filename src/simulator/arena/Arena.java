package simulator.arena;

import java.awt.Color;
import javax.swing.JButton;

import datatypes.Orientation;

/**
 * This Java file handles the simulator arena settings
 */
public class Arena {

    public static final int MAP_WIDTH = 20;
    public static final int MAP_LENGTH = 15;
    private static Arena _instance;
    private Boolean[][] _layout;

    private Arena() {

    }

    /**
     * This function will generate an arena instance.
     *
     * @return
     */
    public static Arena getInstance() {
        if (_instance == null) {
            _instance = new Arena();
        }
        return _instance;
    }

    /**
     * This function will return the arena layout.
     *
     * @return
     */
    public Boolean[][] getLayout() {
        return _layout;
    }

    /**
     * This function will handle the setting up of arena layout.
     * When setting layout, it will take in the mapGrids array and invert the y axis.
     * This is done to save the map in map descriptor much easily
     *
     * @param mapGrids
     */
    public void setLayout(JButton[][] mapGrids) {
        _layout = new Boolean[MAP_LENGTH][MAP_WIDTH];
        for (int x = 0; x < MAP_WIDTH; x++) {
            for (int y = 0; y < MAP_LENGTH; y++) {
                if (mapGrids[x][y].getBackground() == Color.RED) {
                    _layout[y][19 - x] = true;
                } else {
                    _layout[y][19 - x] = false;
                }
            }
        }
    }

    /**
     * This function will return the number of grids that have been explored by the simulator robot.
     * @param sensorPosition
     * @param sensorOrientation
     * @return
     */
    public int getNumOfClearGrids(int[] sensorPosition, Orientation sensorOrientation) {
        int numOfClearGrids = 0;
        switch (sensorOrientation) {
            case NORTH:
                for (int y = sensorPosition[1] + 1; y < Arena.MAP_WIDTH; y++) {
                    if (_layout[sensorPosition[0]][y] == false) {
                        numOfClearGrids++;
                    } else {
                        break;
                    }
                }
                break;
            case SOUTH:
                for (int y = sensorPosition[1] - 1; y >= 0; y--) {
                    if (_layout[sensorPosition[0]][y] == false) {
                        numOfClearGrids++;
                    } else {
                        break;
                    }
                }
                break;
            case EAST:
                for (int x = sensorPosition[0] + 1; x < Arena.MAP_LENGTH; x++) {
                    if (_layout[x][sensorPosition[1]] == false) {
                        numOfClearGrids++;
                    } else {
                        break;
                    }
                }
                break;
            case WEST:
                for (int x = sensorPosition[0] - 1; x >= 0; x--) {
                    if (_layout[x][sensorPosition[1]] == false) {
                        numOfClearGrids++;
                    } else {
                        break;
                    }
                }
        }
        return numOfClearGrids;
    }
}
