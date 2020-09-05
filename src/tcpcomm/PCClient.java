package tcpcomm;

import java.io.IOException;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.Scanner;
import datatypes.Message;

/* This Java file handles the connection settings to RPI */

public class PCClient {
	
	public static final String RPI_IP_ADDRESS = "192.168.101.1";
	public static final int RPI_PORT = 4042;
	private static PCClient _instance;
	private Socket _clientSocket;
	private PrintWriter _toRPi;
	private Scanner _fromRPi;
	
	private PCClient() {
		
	}
	
	//This function will generate a PC instance.
	public static PCClient getInstance() {
        if (_instance == null) {
            _instance = new PCClient();
        }
        return _instance;
    }
	
	//Main function that will setup connection to the RPI.
	//Once connected PC will send a message requesting for the sensor values.
	public static void main (String[] args) throws UnknownHostException, IOException {
		PCClient pcClient = PCClient.getInstance();
		pcClient.setUpConnection(RPI_IP_ADDRESS, RPI_PORT);
		System.out.println("RPI Connected Successfully");
		while (true) {
			pcClient.sendMessage(Message.READ_SENSOR_VALUES);
			String msgReceived = pcClient.readMessage();
			System.out.println("Message Received: "+ msgReceived);
		}	
	}
	
	//This function will handle the connection to RPI
	public void setUpConnection (String IPAddress, int portNumber) throws UnknownHostException, IOException{
		_clientSocket = new Socket(RPI_IP_ADDRESS, RPI_PORT);
		_toRPi = new PrintWriter(_clientSocket.getOutputStream());
		_fromRPi = new Scanner(_clientSocket.getInputStream());
	}
	
	//This function will handle the closing of connection.
	public void closeConnection() throws IOException {
		if (!_clientSocket.isClosed()) {
			_clientSocket.close();
		}
	}
	
	//This function will handle sending message to RPI.
	public void sendMessage(String msg) throws IOException {
		_toRPi.print(msg);
		_toRPi.flush();
		System.out.println("Message Sent: " + msg);
	}

	//This function will handle receiving message to RPI. 
	public String readMessage() throws IOException {
		String messageReceived = _fromRPi.nextLine();
		System.out.println("Message Received: " + messageReceived);
		return messageReceived;
	}
         
}
