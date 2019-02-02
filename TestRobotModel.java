package us.ihmc.valkyrie;

import java.io.InputStream;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel.RobotTarget;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.modelFileLoaders.SdfLoader.DRCRobotSDFLoader;
import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.modelFileLoaders.SdfLoader.JaxbSDFLoader;
import us.ihmc.modelFileLoaders.SdfLoader.RobotDescriptionFromSDFLoader;
import us.ihmc.modelFileLoaders.SdfLoader.SDFContactSensor;
import us.ihmc.modelFileLoaders.SdfLoader.SDFDescriptionMutator;
import us.ihmc.modelFileLoaders.SdfLoader.SDFForceSensor;
import us.ihmc.modelFileLoaders.SdfLoader.SDFJointHolder;
import us.ihmc.modelFileLoaders.SdfLoader.SDFLinkHolder;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFSensor;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.valkyrie.configuration.TestConfigurationRoot;
import us.ihmc.valkyrie.parameters.ValkyrieArmControllerParameters;
import us.ihmc.valkyrie.parameters.ValkyrieCapturePointPlannerParameters;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrie.parameters.ValkyrieSensorInformation;
import us.ihmc.valkyrie.parameters.ValkyrieStateEstimatorParameters;
import us.ihmc.valkyrie.parameters.ValkyrieWalkingControllerParameters;

public class TestRobotModel implements SDFDescriptionMutator {
	/// Member variables
	DRCRobotModel.RobotTarget target = DRCRobotModel.RobotTarget.SCS;
	ValkyrieJointMap jointMap = new ValkyrieJointMap(null);
	
	// Robert
	JaxbSDFLoader loader; 									// robert 1
	RobotDescriptionFromSDFLoader descriptionLoader;		// robert 2 
	RobotDescription robotDescription; 						// robert 3
															// robert 4 (not used, i think?)
	FloatingRootJointRobot sdfRobot;						// robert 5
	SimulationConstructionSet scs;							// robert 6
	private final String[] resourceDirectories;
	{
		resourceDirectories = new String[] { "models/", "models/gazebo/", "models/val_description/",
				"models/val_description/sdf/", };
	}

	public TestRobotModel() {
		// Variables
		InputStream sdf = null;

		// We know model = "DEFAULT" so no if statement
		System.out.println("Loading robot model from: '" + getSdfFile() + "'");
		sdf = getSdfFileAsStream();

		this.loader = DRCRobotSDFLoader.loadDRCRobot(resourceDirectories, sdf, this);	//r1

		// add force sensors to loader
		// for (String forceSensorNames : ValkyrieSensorInformation.forceSensorNames) {
		// RigidBodyTransform transform = new RigidBodyTransform();
		// if (forceSensorNames.equals("leftAnkleRoll") && target != RobotTarget.GAZEBO)
		// {
		// transform.set(
		// ValkyrieSensorInformation.transformFromSixAxisMeasurementToAnkleZUpFrames.get(RobotSide.LEFT));
		// } else if (forceSensorNames.equals("rightAnkleRoll") && target !=
		// RobotTarget.GAZEBO) {
		// transform.set(
		// ValkyrieSensorInformation.transformFromSixAxisMeasurementToAnkleZUpFrames.get(RobotSide.RIGHT));
		// }
		//
		// loader.addForceSensor(jointMap, forceSensorNames, forceSensorNames,
		// transform);
		// }

		// add contact sensors to loader
		// for (RobotSide side : RobotSide.values()) {
		// for (String parentJointName :
		// ValkyrieSensorInformation.contactSensors.get(side).keySet()) {
		// for (String sensorName :
		// ValkyrieSensorInformation.contactSensors.get(side).get(parentJointName)
		// .keySet()) {
		// loader.addContactSensor(jointMap, sensorName, parentJointName,
		// ValkyrieSensorInformation.contactSensors.get(side).get(parentJointName).get(sensorName));
		// }
		// }
		// }

		// do not care (yet?)
		// boolean runningOnRealRobot = target == RobotTarget.REAL_ROBOT;
		// capturePointPlannerParameters = new
		// ValkyrieCapturePointPlannerParameters(runningOnRealRobot);
		// armControllerParameters = new ValkyrieArmControllerParameters(jointMap,
		// target);
		// walkingControllerParameters = new
		// ValkyrieWalkingControllerParameters(jointMap, target);
		// stateEstimatorParamaters = new
		// ValkyrieStateEstimatorParameters(runningOnRealRobot, getEstimatorDT(),
		// sensorInformation, jointMap);

		//instead of calling function "createRobotDescription"
		this.descriptionLoader = new RobotDescriptionFromSDFLoader();	//r2
		this.robotDescription = descriptionLoader.loadRobotDescriptionFromSDF(loader.getGeneralizedSDFRobotModel(jointMap.getModelName())
				, jointMap
				, false); //r3
		
		this.sdfRobot = new FloatingRootJointRobot(robotDescription);
		
	}
	
	public FloatingRootJointRobot getSdfRobot() {
		return this.sdfRobot;
	}

	private String getSdfFile() {
		return TestConfigurationRoot.SIM_SDF_FILE;
	}

	private InputStream getSdfFileAsStream() {
		return getClass().getClassLoader().getResourceAsStream(getSdfFile());
	}

//	private RobotDescription createRobotDescription() {
//		boolean useCollisionMeshes = false;
//
//		GeneralizedSDFRobotModel generalizedSDFRobotModel = getGeneralizedRobotModel();
//		descriptionLoader = new RobotDescriptionFromSDFLoader();
//		this.robotDescription = descriptionLoader.loadRobotDescriptionFromSDF(generalizedSDFRobotModel,
//				jointMap, useCollisionMeshes);
//		return robotDescription;
//	}

	@Override
	public void mutateJointForModel(GeneralizedSDFRobotModel model, SDFJointHolder jointHolder) {
		// TODO Auto-generated method stub

	}

	@Override
	public void mutateLinkForModel(GeneralizedSDFRobotModel model, SDFLinkHolder linkHolder) {
		// TODO Auto-generated method stub

	}

	@Override
	public void mutateSensorForModel(GeneralizedSDFRobotModel model, SDFSensor sensor) {
		// TODO Auto-generated method stub

	}

	@Override
	public void mutateForceSensorForModel(GeneralizedSDFRobotModel model, SDFForceSensor forceSensor) {
		// TODO Auto-generated method stub

	}

	@Override
	public void mutateContactSensorForModel(GeneralizedSDFRobotModel model, SDFContactSensor contactSensor) {
		// TODO Auto-generated method stub

	}

	@Override
	public void mutateModelWithAdditions(GeneralizedSDFRobotModel model) {
		// TODO Auto-generated method stub

	}

	public DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> getDefaultRobotInitialSetup(double groundHeight,
			double initialYaw) {
		// TODO Auto-generated method stub
		return new ValkyrieInitialSetup(groundHeight, initialYaw);
	}
}
