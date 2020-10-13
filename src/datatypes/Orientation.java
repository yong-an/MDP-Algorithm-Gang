package datatypes;

/**
 * This Java file contains the data types for general orientation settings.
 */
public enum Orientation {
	EAST, WEST, SOUTH, NORTH;

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
}
