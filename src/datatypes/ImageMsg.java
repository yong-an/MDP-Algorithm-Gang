package datatypes;

/**
 * inherit from ImageRef
 * this class is used to communicate image information between RPI, algo and android
 */
public class ImageMsg extends ImageRef {
    private int imageId;

    public ImageMsg() {
    }

    public ImageMsg(int imageId) {
        this.imageId = imageId;
    }

    public ImageMsg(ImageRef _imageRef, int imageId) {
        super(_imageRef);
        this.imageId = imageId;
    }

    public ImageMsg(int _x, int _y, Orientation _orientation, int imageId) {
        super(_x, _y, _orientation);
        this.imageId = imageId;
    }

    /**
     * getter for imageId
     * @return image id
     */
    public int getImageId() {
        return imageId;
    }

    /**
     * setter for imageId
     * @param imageId
     */
    public void setImageId(int imageId) {
        this.imageId = imageId;
    }
}
