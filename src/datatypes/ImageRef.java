package datatypes;

import java.util.Comparator;
import simulator.arena.Arena;

public class ImageRef implements Comparable<ImageRef> {
	private int x, y;
	private Orientation orientation;

	public ImageRef() {
		x = 0;
		y = 0;
		orientation = orientation.NORTH;
	}
	
	public ImageRef(ImageRef _imageRef) {
		x = _imageRef.getX();
		y = _imageRef.getY();
		orientation = _imageRef.getOrientation();
	}

	public ImageRef(int _x, int _y, Orientation _orientation) {
		x = _x;
		y = _y;
		orientation = _orientation;
	}

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
}
