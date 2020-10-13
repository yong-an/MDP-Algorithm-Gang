package datatypes;

/**
 * This Java file contains the data types for general orientation settings.
 */
public enum Orientation {
	EAST, WEST, SOUTH, NORTH;

	/**
	 * convert input string of orientation into Orientation
	 * @param s expected input string = NORTH | SOUTH | WEST | EAST
	 * @return Orientation object
	 */
	public static Orientation getOrientationFromStr(String s){
		s = s.toUpperCase();
		switch(s)
		{
			case "NORTH":
				return NORTH;
			case "SOUTH":
				return SOUTH;
			case "WEST":
				return WEST;
			case "EAST":
				return EAST;
		}
		return null;
	}

	/**
	 * convert input Orientation into its corresponding letter
	 * @param o
	 * @return a string of length 1: N | S | W | E
	 */
	public static String getOrientationLetter(Orientation o){
		switch(o)
		{
			case NORTH:
				return "N";
			case SOUTH:
				return "S";
			case WEST:
				return "W";
			case EAST:
				return "E";
		}
		return "";
	}
}
