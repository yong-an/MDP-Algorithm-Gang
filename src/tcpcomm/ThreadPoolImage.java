package tcpcomm;

import datatypes.ImageMsg;
import datatypes.Orientation;

import java.io.IOException;
import java.nio.file.*;
import java.util.ArrayList;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import static java.nio.file.StandardWatchEventKinds.ENTRY_CREATE;

public class ThreadPoolImage implements Runnable {
    private static ArrayList<ImageMsg> imageMsgList;
    private static PCClient pcClient;

    @Override
    public void run() {
        System.out.println("Thread for new images started");

        imageMsgList = new ArrayList<ImageMsg>();
        pcClient = PCClient.getInstance();

        try {
            poolDirectory();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    /**
     * watch folder for created files
     *
     * @throws IOException
     * @throws InterruptedException
     */
    private void poolDirectory() throws IOException, InterruptedException { // todo watch folder
        WatchService watcher = FileSystems.getDefault().newWatchService();
//		Path dir = Paths.get("D:\\Sample"); // todo update pooling folder path
        Path dir = Paths.get("F:\\Sample");
        WatchKey key = dir.register(watcher, ENTRY_CREATE);

        while ((key = watcher.take()) != null) {
            for (WatchEvent<?> event : key.pollEvents()) {
                System.out.println("New File: " + event.kind() + " - " + event.context());

                // consider file only at ENTRY_CREATE
                if (event.kind() == ENTRY_CREATE) {
                    imageMsgList.add(fileNameToImageMsg(event.context().toString()));

                    // update algo stimulator todo


                    // send msg to android
                    String msgToAndroid = imageMsgListToString(imageMsgList);
                    System.out.println("Msg to be send to Android: " + msgToAndroid);
//                    pcClient.sendMessageToAndroid(msgToAndroid); // todo need test if it works
                }
            }

            key.reset();
        }
    }

    /**
     * convert string of new image file name to ImageMsg
     *
     * @param fileName string of new image file
     * @return ImageMsg with image info
     */
    private ImageMsg fileNameToImageMsg(String fileName) {
        ImageMsg image = new ImageMsg();
        String msg = "";
        String o = "";
        String str_digits = "";

        Pattern patternAll = Pattern.compile("\\d+,\\d+,\\d+,(NORTH|SOUTH|EAST|WEST)");
        Matcher matcher1 = patternAll.matcher(fileName);
        if (matcher1.find()) {
            msg = matcher1.group(); // matched string eg "13,5,4,EAST"
            o = matcher1.group(1); // NORTH|SOUTH|EAST|WEST
            image.setOrientation(Orientation.getOrientationFromStr(o));

            String[] digits = msg.split(",");
            image.setImageId(Integer.parseInt(digits[0])); // image id
            image.setX(Integer.parseInt(digits[1])); // x-coord
            image.setY(Integer.parseInt(digits[2])); // y-coord

        }

        return image;
    } //todo confirm image filename

    /**
     * convert all found images info, into a string of format [imageID,x-coord,y-coord]
     * @param imageMsgList
     * @return string with format [[imageID,x-coord,y-coord],[imageID,x-coord,y-coord]]
     */
    private String imageMsgListToString(ArrayList<ImageMsg> imageMsgList) {
        String msg = "[";

        for (ImageMsg i : imageMsgList) {
            String s = "[" + i.getImageId() + "," + i.getX() + "," + i.getY() + "],";
            msg += s;
        }

        msg = msg.substring(0, msg.length()-1); // remove last ,
        msg += "]";
        return msg;
    }
}
