package us.ihmc.demo;

import java.io.IOException;
import java.text.DecimalFormat;
import java.util.Arrays;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.jnativehook.GlobalScreen;
import org.jnativehook.NativeHookException;
import org.jnativehook.keyboard.NativeKeyEvent;
import org.jnativehook.keyboard.NativeKeyListener;

import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.valkyrie.TestRobotModel;
import us.trec.easyCAT.easyCATController;

public class DarwinDemo implements NativeKeyListener {

	// Make scs and easyCAT available for all methods
	static SimulationConstructionSet scs;
	static easyCATController easyCAT1;

	// Get chosen joint
	DecimalFormat format = new DecimalFormat("##.00"); // rounding to 2 decimal places
	static double JOINT_STEP_SIZE = 0.05;
	static boolean IS_JOINT_CHOSEN = false;
	static String[] CHOSEN_JOINTS = { "q_j_tilt", "q_j_high_arm_l", "q_j_high_arm_r", "q_j_gripper_l", "q_j_gripper_r",
			"q_j_tibia_l", "q_j_tibia_r", "q_j_thigh2_l", "q_j_thigh2_r", "q_j_pan" };
	static double[] JOINTS_VALUES = new double[CHOSEN_JOINTS.length];
	// specific to CHOSEN_JOINTS
	static boolean[] REVERSED_DIRS = { false, true, true, false, false, false, true, false, true, false };
	static int JOINT_CHOSEN = 1; // joint chosen at beginning when motor is manually moved
	// TODO: Fill remaining joint limits
	final static double[] ABS_JOINT_LIM = { 0, 1.6, 0, 0, 0, 0, 0, 0, 0, 0 }; // the limit of the joint (+-)

	// code for keyboard presses
	final int UP_ARROW = 57416, DOWN_ARROW = 57424, SPACE = 57, SHIFT_L = 42, SHIFT_R = 54, T = 20;
	final boolean MOVE_UP = true, MOVE_DOWN = false;

	// Target position handler
	static double TARGET_POS;
	static String TARGET_COMMAND = "";

	// Handle loosening motor after arrow presses
	static int INCREMENT_TIMER = 0, TIMER_MAX = 150000;
	static boolean TURNON = false;

	// ARBITER
	public enum Arbiter {
		INCREMENT, /* TARGET, */ READ, NONE,
	}

	static Arbiter inCharge = Arbiter.READ;

	// Threads
	static Runnable updateMotors = new Runnable() {
		public void run() {
			handleEtherCAT();
		}
	};

	static Thread update = new Thread(updateMotors); // Thread to deal with motor reads

	/*
	 * 
	 */
	public static void main(String[] args) throws IOException {
		// Listen to user input
		try {
			GlobalScreen.registerNativeHook();
		} catch (NativeHookException e) {
			e.printStackTrace();
		}
		GlobalScreen.getInstance().addNativeKeyListener(new DarwinDemo());
		Logger logger = Logger.getLogger(GlobalScreen.class.getPackage().getName());
		logger.setLevel(Level.OFF);
		logger.setUseParentHandlers(false);
		// end of listening to user input

		// Create robot
		TestRobotModel robotModel = new TestRobotModel();

		// Set all position to 0
		Arrays.fill(JOINTS_VALUES, 0.0);

		// Parameters for Simulation Environment
		SimulationConstructionSetParameters scsParameters = new SimulationConstructionSetParameters();
		scs = new SimulationConstructionSet(robotModel.getSdfRobot(), scsParameters);
		scs.setGroundVisible(false);

		// Set a nice camera angle
		scs.setCameraFix(0.0168, 0.0435, -0.1556);
		scs.setCameraPosition(2.3212, -0.6235, 0.0574);

		// Start simulation
		scs.startOnAThread();
		startEtherCAT(); // NO INCREMENT
	}

	/*
	 * 
	 */
	public static void handleEtherCAT() {
		while (true) {
			switch (inCharge) {
			case INCREMENT:
				easyCAT1.writeMode();
				INCREMENT_TIMER++;
				if (INCREMENT_TIMER > TIMER_MAX) {
					inCharge = Arbiter.READ;
					sendTurnOnSignal(false);
					easyCAT1.quickSend();		//makes sure change is sent before read mode
				}
				break;
			case NONE:
				break;
			case READ:
				easyCAT1.readMode();
				int real_position = easyCAT1.getATRPos(0);
				double scs_position = byteToSCSposition(real_position);
//				System.out.println(real_position);
				setRobotPosition(scs_position);
				INCREMENT_TIMER = 0;
				break;
//			case TARGET:
//				break;
			default:
				break;
			}
			
			System.out.println(scs.getVariable(CHOSEN_JOINTS[JOINT_CHOSEN]).getValueAsDouble());
		}
		
	}

	/*
	 * This function takes a byte it receives from etherCAT 'eth_byte' and converts
	 * it to a position in SCS-units based on the chosen joint's limits (positive
	 * and negative)
	 */
	static double byteToSCSposition(int eth_byte) {
		double scs_pos = (-2 * (double) eth_byte * ABS_JOINT_LIM[JOINT_CHOSEN] / 255.0 + ABS_JOINT_LIM[JOINT_CHOSEN]);
		// System.out.println(scs_pos);
		return scs_pos;
	}

	/*
	 * This function takes an SCS position 'scs_pos' and converts it to a byte to be
	 * sent via etherCAT
	 */
	static int SCSpositionToByte(double scs_pos) {
		double eth_byte = 255 * (scs_pos - ABS_JOINT_LIM[JOINT_CHOSEN]) / (-2 * ABS_JOINT_LIM[JOINT_CHOSEN]);
		return (int) eth_byte;
	}

	/*
	 * 
	 */
	private static void startEtherCAT() throws IOException {
		// create easyCATtry to start communication
		int[] message = { 0, 0 };
		easyCAT1 = new easyCATController(message);

		// COMMUNICATION STARTING POINT
		easyCAT1.start();
		update.start();
		easyCAT1.join();
	}

	/*
	 * This function sets a joint position in the SCS simulation based on a given
	 * target. The chosen joint is decided before entering the function
	 */
	private static void setRobotPosition(double target) {
		YoVariable<?> joint = scs.getVariable(CHOSEN_JOINTS[JOINT_CHOSEN]);
		joint.setValueFromDouble(target);
	}

	/*
	 * This function sets a joint position in the SCS simulation based on a given
	 * target and a given joint.
	 */
	private static void setRobotPosition(double target, int joint) {
		JOINT_CHOSEN = joint;
		setRobotPosition(target);
	}

	/*
	 * 
	 */
	private void handleMovement(int jointChosen, boolean dir) {
		inCharge = Arbiter.INCREMENT;
		INCREMENT_TIMER = 0; // prevent timer from incrementing since motor is moving
		// set turnON bit on to prevent motor from loosening
		sendTurnOnSignal(true);

		// logic to set joint movement direction
		YoVariable<?> joint = scs.getVariable(CHOSEN_JOINTS[jointChosen]); // get joint
		boolean reversed = REVERSED_DIRS[jointChosen]; // is joint reversed in scs?
		int direction = ((dir && !reversed) || (!dir && reversed)) ? 1 : 0; // make sure up goes up

		// get limits for chosen joint
		double targetPos = 0.0;
		double targetLimit = ABS_JOINT_LIM[jointChosen];

		// move joints
		switch (direction) {
		case 1: // UP
			targetPos = joint.getValueAsDouble() + JOINT_STEP_SIZE;
			if (targetPos > targetLimit)
				targetPos = targetLimit;

			joint.setValueFromDouble(targetPos);
			break;
		case 0: // DOWN
			targetPos = joint.getValueAsDouble() - JOINT_STEP_SIZE;
			if (targetPos < -targetLimit)
				targetPos = -targetLimit;

			joint.setValueFromDouble(targetPos);
			break;

		default:
			System.out.println("SHOULD NOT GET HERE.");
			break;
		}

		// update values in array
		JOINTS_VALUES[jointChosen] = Double.parseDouble(format.format(targetPos));
		
		// send values via etherCAT
		easyCAT1.setArrayPos(0, SCSpositionToByte(targetPos));
	}

	/*
	 * This function sends a signal to loosen the motor when 'tf' is false, and
	 * locks the motor into position when 'tf' is true
	 */
	public static void sendTurnOnSignal(boolean tf) {
		if (tf) {
			// turn motor on (locks)
			easyCAT1.setArrayPos(1, 1);
		} else
			// turn motor off (loosen)
			easyCAT1.setArrayPos(1, 0);

		TURNON = tf;
	}

	// ----------------------------------------------------------------------------------------------------
	@Override
	public void nativeKeyPressed(NativeKeyEvent e) {
		// Choosing which part to move
		try {
			JOINT_CHOSEN = (int) Double.parseDouble(NativeKeyEvent.getKeyText(e.getKeyCode()));
			inCharge = Arbiter.READ;
			System.out.println("Joint " + JOINT_CHOSEN + ": " + CHOSEN_JOINTS[JOINT_CHOSEN] + " chosen.");

		} catch (Exception e2) { // when key pressed is not a number
			// Choosing which direction to move joint

			// Handle movement: True = up | False = down
			if (e.getKeyCode() == UP_ARROW)
				handleMovement(JOINT_CHOSEN, MOVE_UP);
			else if (e.getKeyCode() == DOWN_ARROW)
				handleMovement(JOINT_CHOSEN, MOVE_DOWN);
			// else if (e.getKeyCode() == SHIFT_R || e.getKeyCode() == SHIFT_L) {
			// inCharge = Arbiter.TARGET;
			// System.out.print("Set " + CHOSEN_JOINTS[JOINT_CHOSEN] + " position to: ");
			// }
		}

	}

	@Override
	public void nativeKeyReleased(NativeKeyEvent arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void nativeKeyTyped(NativeKeyEvent arg0) {
		// TODO Auto-generated method stub

	}

}
