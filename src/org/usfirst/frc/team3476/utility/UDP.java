package org.usfirst.frc.team3476.utility;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import org.json.simple.JSONObject;
import org.json.simple.JSONValue;
import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.utility.Threaded;
import org.usfirst.frc.team3476.utility.math.Rotation;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Class that handles sockets for receiving and sending. It doesn't release
 * sockets and keeps them for later use. There is one worker thread for handling
 * messages. The buffer is 2048 bytes long.
 */
public class UDP extends Threaded {

	private class MessageHandler extends Threaded {

		DatagramPacket packet;

		public MessageHandler(DatagramPacket packet) {
			this.packet = packet;
		}

		@Override
		public void update() {
			String rawMessage = new String(packet.getData(), 0, packet.getLength());
			// TODO: do whatever with message bruh
			JSONObject message = (JSONObject) JSONValue.parse(rawMessage);
		}
	}

	private static final UDP instance = new UDP();

	public static UDP getInstance() {
		return UDP.instance;
	}

	private ExecutorService workers;

	private DatagramSocket listener;
	private HashMap<Integer, DatagramSocket> senders;

	private UDP() {
		senders = new HashMap<Integer, DatagramSocket>();
		try {
			listener = new DatagramSocket(5800);
		} catch (SocketException e) {
			e.printStackTrace();
		}
		workers = Executors.newFixedThreadPool(1);
	}

	@Override
	public void update() {
		byte[] buffer = new byte[2048];
		DatagramPacket msg = new DatagramPacket(buffer, buffer.length);
		try {
			listener.receive(msg);
			workers.execute(new MessageHandler(msg));
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	/**
	 * 
	 * @param addr
	 * 			Address to send message to
	 * @param message
	 * 			Contents of UDP packets
	 * @param port
	 * 			Port to send message over 
	 */
	public void send(String addr, String message, int port) {
		if (!senders.containsKey(port)) {
			try {
				senders.put(port, new DatagramSocket(port));
			} catch (SocketException e) {
				e.printStackTrace();
			}
		}
		DatagramPacket msg = null;
		try {
			msg = new DatagramPacket(message.getBytes(), message.getBytes().length, InetAddress.getByName(addr), port);
			senders.get(port).send(msg);
		} catch (UnknownHostException e1) {
			e1.printStackTrace();
			System.out.println("Host:" + addr + " not found!");
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

}
