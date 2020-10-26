package tcpcomm;

import datatypes.ImageMsg;
import datatypes.Orientation;
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
//		Path dir = Paths.get("D:\\Sample"); // todo update pooling folder path
        Path dir = Paths.get("D:\\Downloads\\TrainYourOwnYOLO-master\\Test");
        WatchKey key = dir.register(watcher, ENTRY_CREATE);

        while ((key = watcher.take()) != null) {
            for (WatchEvent<?> event : key.pollEvents()) {
                System.out.println("New File: " + event.kind() + " - " + event.context());

                // consider file only at ENTRY_CREATE
                if (event.kind() == ENTRY_CREATE) {
                    ImageMsg image = fileNameToImageMsg(event.context().toString());
//                    imageMsgList.add(image);

                    // update algo stimulator, show image info on stimulator
//                    controller.foundImage(image.getTargetX(), image.getTargetY(), image.getImageId(), Orientation.getOrientationLetter(image.getOrientation()));

                    // send msg to android
                    String msgToAndroid = imageMsgListToString(image);
                    System.out.println("Msg to be send to Android: " + msgToAndroid);
                    pcClient.sendMessageToAndroidPic(msgToAndroid); // todo need test if it works
                }
            }

            key.reset();
        }
    }

    /**
     * convert string of new image file name to ImageMsg
     * expected file name = robot-x,robot-y,robot-orien,img-id,img-x,img-y,img-orien
     *
     * @param fileName string of new image file
     * @return ImageMsg with image info
     */
    private ImageMsg fileNameToImageMsg(String fileName) {
        ImageMsg image = new ImageMsg();

        Pattern patternAll = Pattern.compile("\\d+,\\d+,\\d+,(NORTH|SOUTH|EAST|WEST),\\d+,\\d+,(NORTH|SOUTH|EAST|WEST)");
        Matcher matcher1 = patternAll.matcher(fileName);
        if (matcher1.find()) {
        	// 12161016,12,18,WEST,10,19,SOUTH!.png
            String msg = matcher1.group(); // matched string eg "1,2,NORTH,13,5,4,EAST"
//            System.out.println(msg);
            String robotOrien = matcher1.group(1); // 1st NORTH|SOUTH|EAST|WEST
            String imageOrien = matcher1.group(2); // 2nd NORTH|SOUTH|EAST|WEST

            String[] splitMsgs = msg.split(",");
//            image.setX(Integer.parseInt(splitMsgs[0])); // robot-x
//            image.setY(Integer.parseInt(splitMsgs[1])); // robot-y
//            image.setOrientation(Orientation.getOrientationFromStr(robotOrien)); // robot-orientation
            image.setImageId(Integer.parseInt(splitMsgs[0])); // image id
            image.setTargetX(Integer.parseInt(splitMsgs[4]));// image-x
            image.setTargetY(Integer.parseInt(splitMsgs[5]));// image-y
            image.setTargetOrientation(Orientation.getOrientationFromStr(imageOrien));// image-orientation
//            System.out.println(splitMsgs[0]);
//            System.out.println(splitMsgs[4]);
//            System.out.println(splitMsgs[5]);
//            System.out.println(imageOrien);
            
        }

        return image;
    } //todo RPI pls confirm image filename

    /**
     * convert all found images info, into a string of format [imageID,x-coord,y-coord]
     * @param imageMsgList
     * @return string with format [[imageID,x-coord,y-coord],[imageID,x-coord,y-coord]]
     */
    private String imageMsgListToString(ImageMsg i) {
//        String msg = "[";

//        for (ImageMsg i : imageMsgList) {
//            String s = "[" + i.getImageId() + "," + i.getTargetX() + "," + i.getTargetY() + "],";
//            msg += s;
//        }
        
        String s = "[" + i.getImageId() + "," + i.getTargetX() + "," + i.getTargetY() + "]";

//        msg = msg.substring(0, msg.length()-1); // remove last ,
//        msg += "]";
//        return msg;
        return s;
    }
}
