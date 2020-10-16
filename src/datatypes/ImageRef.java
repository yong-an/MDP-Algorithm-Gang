package datatypes;

import java.util.Comparator;
import simulator.arena.Arena;

public class ImageRef implements Comparable<ImageRef> {
	// robot's
	private int x, y;
	private Orientation orientation;
	// obstacle's
	private int targetX, targetY;
	private Orientation targetOrientation;
	
	public ImageRef() {
		x = 0;
		y = 0;
		orientation = orientation.NORTH;
		setTargetX(0);
		setTargetY(0);
		targetOrientation = orientation.SOUTH;
	}
	
	public ImageRef(ImageRef _imageRef) {
		x = _imageRef.getX();
		y = _imageRef.getY();
		orientation = _imageRef.getOrientation();
		targetX = _imageRef.getTargetX();
		targetY = _imageRef.getTargetY();
		targetOrientation = _imageRef.getTargetOrientation();
	}

	public ImageRef(int _x, int _y, Orientation _orientation, int _targetX, int _targetY, Orientation _targetOrientation) {
		x = _x;
		y = _y;
		orientation = _orientation;
		targetX = _targetX;
		targetY = _targetY;
		targetOrientation = _targetOrientation;
	}
	
	public ImageRef(int _x, int _y, Orientation _orientation) {
		x = _x;
		y = _y;
		orientation = _orientation;
		targetX = 0;
		targetY = 0;
		targetOrientation = orientation.NORTH;
	}
	
	public int distanceFromTarget()
	{
		if (Math.abs(x - targetX) == 2 || Math.abs(y - targetY) == 2) 
			return 0;
		return 1;
	}
	
	public boolean isThereAdjacentImageRef(int _x, int _y, Orientation _adjacentDirection)
	{
		//out of bounds guard
		if (_x >= Arena.MAP_LENGTH || _x < 0 || _y >= Arena.MAP_WIDTH || _y < 0)
			return false;
		switch(_adjacentDirection)
		{
		case NORTH:
			if (x == _x && y == _y-1)
				return true;
			break;
		case SOUTH:
			if (x == _x && y == _y+1)
				return true;
			break;
		case EAST:
			if (x == _x-1 && y == _y)
				return true;
			break;
		case WEST:
			if (x == _x+1 && y == _y)
				return true;
			break;
			default:
			return false;
		}
		return false;
	}
	
	public boolean targetIsEqual(ImageRef _imageRef)
	{
		if (targetX == _imageRef.getTargetX() && targetY == _imageRef.getTargetY() && targetOrientation.equals(_imageRef.getTargetOrientation()))
		{
			return true;
		}
		return false;
	}

	public boolean equalsAbsolute(Object object) {
		if (object == null || object.getClass() != getClass()) {
			return false;
		} else {
			ImageRef method = (ImageRef) object;
			if (this.x == method.getX() && this.y == method.getY() && this.orientation.equals(method.getOrientation())
					&& this.targetX == method.getTargetX() && this.targetY == method.getTargetY() && this.targetOrientation.equals(method.getTargetOrientation())) {
					return true;
			}
		}
		return false;
	}
	
	//ignores target
	@Override
	public boolean equals(Object object) {
		if (object == null || object.getClass() != getClass()) {
			return false;
		} else {
			ImageRef method = (ImageRef) object;
			if (this.x == method.getX() && this.y == method.getY() && this.orientation.equals(method.getOrientation())) {
				return true;
			}
		}
		return false;
	}

	@Override
	public int hashCode() {
		int hash = 3;
		hash = 7 * hash + this.x;
		hash = 7 * hash + this.y;
		hash = 7 * hash + (orientation == null ? 0 : orientation.hashCode());
		return hash;
	}

	public int compareTo(ImageRef _that) {

        return Integer.compare(x + y*Arena.MAP_WIDTH-1, _that.x + _that.y*Arena.MAP_WIDTH-1);
    }
	
	public int getX() {
		return x;
	}

	public void setX(int x) {
		this.x = x;
	}

	public int getY() {
		return y;
	}

	public void setY(int y) {
		this.y = y;
	}

	public Orientation getOrientation() {
		return orientation;
	}

	public void setOrientation(Orientation orientation) {
		this.orientation = orientation;
	}

	public int getTargetX() {
		return targetX;
	}

	public void setTargetX(int targetX) {
		this.targetX = targetX;
	}

	public int getTargetY() {
		return targetY;
	}

	public void setTargetY(int targetY) {
		this.targetY = targetY;
	}

	public Orientation getTargetOrientation() {
		return targetOrientation;
	}

	public void setTargetOrientation(Orientation targetOrientation) {
		this.targetOrientation = targetOrientation;
	}
}
