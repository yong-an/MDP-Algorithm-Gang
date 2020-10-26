package tcpcomm;

import datatypes.ImageMsg;
import datatypes.Orientation;
import main.RobotSystem;
import simulator.Controller;

import java.io.IOException;
import java.nio.file.*;
import java.util.ArrayList;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import static java.nio.file.StandardWatchEventKinds.ENTRY_CREATE;

/**
 * thread class to pool image files from a folder
 * read image file name to obtain image info
 * update algo stimulator with the image info
 * and send image info to Android
 */
public class ThreadPoolImage implements Runnable {
    private static ArrayList<ImageMsg> imageMsgList;
    private static PCClient pcClient;
    private static Controller controller;

    @Override
    public void run() {
        System.out.println("Thread for new images started");

        imageMsgList = new ArrayList<ImageMsg>();
        pcClient = PCClient.getInstance();
        controller = Controller.getInstance();

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
    private void poolDirectory() throws IOException, InterruptedException {
        WatchService watcher = FileSystems.getDefault().newWatchService();
//		Path dir = Paths.get("D:\\Sample");
        Path dir = Paths.get("D:\\Downloads\\TrainYourOwnYOLO-master\\Test");
        WatchKey key = dir.register(watcher, ENTRY_CREATE);

        while ((key = watcher.take()) != null) {
            for (WatchEvent<?> event : key.pollEvents()) {
                System.out.println("New File: " + event.kind() + " - " + event.context());

                // consider file only at ENTRY_CREATE
                if (event.kind() == ENTRY_CREATE) {
                    ImageMsg image = fileNameToImageMsg(event.context().toString());

                    // send msg to android
                    String msgToAndroid = imageMsgToString(image);
                    System.out.println("Msg to be send to Android: " + msgToAndroid);
                    if(RobotSystem.isRealRun()){
                        // send msg to Android in real run
                        pcClient.sendMessageToAndroidPic(msgToAndroid);
                    }
                }
            }

            key.reset();
        }
    }

    /**
     * convert string of new image file name to ImageMsg
     * expected file name = img-id,robot-x,robot-y,robot-orien,img-x,img-y,img-orien
     *
     * @param fileName string of new image file
     * @return ImageMsg with image info
     */
    private ImageMsg fileNameToImageMsg(String fileName) {
        ImageMsg image = new ImageMsg();

        Pattern patternAll = Pattern.compile("\\d+,\\d+,\\d+,(NORTH|SOUTH|EAST|WEST),\\d+,\\d+,(NORTH|SOUTH|EAST|WEST)");
        Matcher matcher1 = patternAll.matcher(fileName);
        if (matcher1.find()) {
            String msg = matcher1.group(); // matched string eg "13,1,2,NORTH,5,4,EAST"
//            System.out.println(msg);
            String imageOrien = matcher1.group(2); // 2nd NORTH|SOUTH|EAST|WEST

            String[] splitMsgs = msg.split(",");
            image.setImageId(Integer.parseInt(splitMsgs[0])); // image id
            image.setTargetX(Integer.parseInt(splitMsgs[4]));// image-x
            image.setTargetY(Integer.parseInt(splitMsgs[5]));// image-y
            image.setTargetOrientation(Orientation.getOrientationFromStr(imageOrien));// image-orientation
            
        }

        return image;
    }


    /**
     * convert all found images info, into a string of format [imageID,x-coord,y-coord]
     * @param i ImageMsg
     * @return string of format [imageID,x-coord,y-coord]
     */
    private String imageMsgToString(ImageMsg i) {
        String s = "[" + i.getImageId() + "," + i.getTargetX() + "," + i.getTargetY() + "]";
        return s;
    }
}
