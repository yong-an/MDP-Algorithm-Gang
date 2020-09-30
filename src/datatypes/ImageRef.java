package datatypes;

import simulator.arena.Arena;

public class ImageRef {
	private int x, y;
	private Orientation orientation;

	public ImageRef() {
		x = 0;
		y = 0;
		orientation = orientation.NORTH;
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
