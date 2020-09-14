package tcpcomm;

import java.io.IOException;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.Scanner;
import java.time.LocalTime;
import datatypes.Message;

/**
 * This Java file handles the connection settings to RPI
 */
public class PCClient {

    public static final String RPI_IP_ADDRESS = "192.168.23.23";
    public static final int RPI_PORT = 22;
    private static PCClient _instance;
    private Socket _clientSocket;
    private PrintWriter _toRPi;
    private Scanner _fromRPi;

    private PCClient() {

    }


    /**
     * //This function will generate a PC instance.
     *
     * @return
     */
    public static PCClient getInstance() {
        if (_instance == null) {
            _instance = new PCClient();
        }
        return _instance;
    }

    /**
     * Main function that will setup connection to the RPI.
     * Once connected PC will send a message requesting for the sensor values.
     *
     * @param args
     * @throws UnknownHostException
     * @throws IOException
     */
    public static void main(String[] args) throws UnknownHostException, IOException {
        PCClient pcClient = PCClient.getInstance();
        pcClient.setUpConnection(RPI_IP_ADDRESS, RPI_PORT);
        System.out.println("RPI Connected Successfully");
        while (true) {
            pcClient.sendMessage(Message.READ_SENSOR_VALUES);
            String msgReceived = pcClient.readMessage();
            System.out.println("Message Received: " + msgReceived);
        }
    }

    /**
     * This function will handle the connection to RPI
     *
     * @param IPAddress
     * @param portNumber
     * @throws UnknownHostException
     * @throws IOException
     */
    public void setUpConnection(String IPAddress, int portNumber) throws UnknownHostException, IOException {
        _clientSocket = new Socket(RPI_IP_ADDRESS, RPI_PORT);
        _toRPi = new PrintWriter(_clientSocket.getOutputStream());
        _fromRPi = new Scanner(_clientSocket.getInputStream());
    }

    /**
     * This function will handle the closing of connection.
     *
     * @throws IOException
     */
    public void closeConnection() throws IOException {
        if (!_clientSocket.isClosed()) {
            _clientSocket.close();
        }
    }

    /**
     * This function will handle sending message to Arduino.
     *
     * @param msg
     * @throws IOException
     */
    public void sendMessage(String msg) throws IOException {

		msg = "@s"+msg+"!";
		_toRPi.print(msg);
		_toRPi.flush();
		
		System.out.println("Message sent to Arduino: " + msg+" - "+LocalTime.now());
	}
    /**
     * This function will handle sending message to Android.
     *
     * @param msg
     */
	public void sendMessageToAndroid(String msg) {
		msg = "@b"+msg+"!";
		_toRPi.print(msg);
		_toRPi.flush();
		
		System.out.println("Message sent to Android: " + msg+" - "+LocalTime.now());
	}
	
    /**
     * This function will handle sending message to R-PI.
     *
     * @param msg
     */
	public void sendMsgToRPI(String msg){
		msg = "@r"+msg+"!";
		_toRPi.print(msg);
		_toRPi.flush();
		
		System.out.println("Message sent to RPI: " + msg+" - "+LocalTime.now());
	}
    /**
     * This function will handle receiving message from RPI.
     *
     * @return
     * @throws IOException
     */
    public String readMessage() throws IOException {
        String messageReceived = _fromRPi.nextLine();
        System.out.println("Message Received: " + messageReceived);
        return messageReceived;
    }

}
