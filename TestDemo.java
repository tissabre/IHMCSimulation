package us.ihmc.demo;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.valkyrie.TestRobotModel;

public class TestDemo {

	public static void main(String[] args) {
		TestRobotModel robotModel = new TestRobotModel();

		
		// Robert 6
		SimulationConstructionSetParameters scsParameters = new SimulationConstructionSetParameters();
		SimulationConstructionSet scs = new SimulationConstructionSet(robotModel.getSdfRobot(), scsParameters);

	}
}
