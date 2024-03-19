
import java.math.BigDecimal

import javax.xml.bind.JAXBException

import org.mujoco.MuJoCoModelManager
import org.mujoco.xml.Mujoco
import org.mujoco.xml.Mujoco.Actuator.Builder
import org.mujoco.xml.attributetypes.IntegratorType
import org.mujoco.xml.attributetypes.JointtypeType
import org.mujoco.xml.body.JointType

import com.neuronrobotics.bowlerstudio.BowlerStudio
import com.neuronrobotics.bowlerstudio.BowlerStudioController
import com.neuronrobotics.bowlerstudio.creature.MobileBaseCadManager
import com.neuronrobotics.bowlerstudio.physics.MuJoCoPhysicsManager
import com.neuronrobotics.bowlerstudio.physics.TransformFactory
import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine
import com.neuronrobotics.sdk.addons.kinematics.AbstractLink
import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.LinkConfiguration
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR
import com.neuronrobotics.sdk.common.DeviceManager
import com.neuronrobotics.sdk.common.IDeviceProvider

import eu.mihosoft.vrl.v3d.CSG
import eu.mihosoft.vrl.v3d.Transform
import javafx.scene.transform.Affine

public class MyManager extends com.neuronrobotics.bowlerstudio.physics.MuJoCoPhysicsManager{

	public MyManager(String name,List<MobileBase> bases, List<CSG> freeObjects,
	List<CSG> fixedObjects,	File workingDir){
		super(name,bases,freeObjects,fixedObjects,workingDir);
	}
	private boolean checkLinkPhysics(MobileBaseCadManager cadMan,LinkConfiguration conf) {
		ArrayList<CSG>  parts=cadMan.getLinktoCadMap().get(conf);
		for(CSG c:parts) {
			if(checkForPhysics(c))
				return true;
		}
		return false;
	}
	@Override
	public void loadBase(MobileBase cat, Builder<?> actuators) throws IOException {
		if(contacts==null)
			contacts= builder.addContact();
		String bodyName = getMujocoName(cat);
		MobileBaseCadManager cadMan = MobileBaseCadManager.get(cat);
		loadCadForMobileBase(cadMan);
		int bodyParts=0;
		ArrayList<CSG> arrayList = cadMan.getBasetoCadMap().get(cat);
		TransformNR center = cat.getCenterOfMassFromCentroid();
		String centerString = center.getX()/1000.0+" "+
				center.getY()/1000.0+" "+
				center.getZ()/1000.0+" ";
		org.mujoco.xml.BodyarchType.Builder<?> addBody = addWorldbody.addBody()
				.withName(bodyName);
		//		addBody.addInertial()
		//				.withMass(BigDecimal.valueOf(cat.getMassKg()/1000.0))
		//				.withPos(centerString)
		//				.withDiaginertia("1 1 1" );
		addBody.addFreejoint();
		//setStartLocation(center, addBody);
		for (int i = 0; i < arrayList.size(); i++) {
			CSG part = arrayList.get(i);

			if(!checkForPhysics(part))
				continue;
			bodyParts++;
			String nameOfCSG = bodyName+"_CSG_"+bodyParts;
			CSG transformed = part.transformed(TransformFactory.nrToCSG(center).inverse());
			CSG hull = transformed.hull();
			transformed=part.clone();
			transformed.setManipulator(new Affine());
			putCSGInAssets(nameOfCSG, hull,true);
			org.mujoco.xml.body.GeomType.Builder<?> geom = addBody.addGeom();
			ArrayList<CSG> parts = getMapNameToCSGParts(bodyName);
			parts.add( transformed);
			setCSGMeshToGeom(nameOfCSG, geom);
			geom.withPos(centerString);
		}
		for(DHParameterKinematics l:cat.getAllDHChains()) {
			if(l.getScriptingName().contains("Dummy"))
				continue;
			String lastName = bodyName;
			for(int i=0;i<l.getNumberOfLinks();i++) {

				AbstractLink link = l.getAbstractLink(i);
				LinkConfiguration conf = link.getLinkConfiguration();
				if(!checkLinkPhysics(cadMan,conf))
					continue;
				String name = conf.getName()+"_"+l.getScriptingName();
				//println "Loading link "+name
				mapNameToLink.put(name, link);
				Affine a = (Affine)link.getGlobalPositionListener();
				//println "Link listener "+s+" is "+a
				affineNameMap.put(a, name);
				contacts.addExclude()
						.withBody1(bodyName)
						.withBody2(name);
				contacts.addExclude()
						.withBody1(lastName)
						.withBody2(name);
				lastName=name;
			}
		}

		for(DHParameterKinematics k:cat.getAllDHChains()) {
			if(k.getScriptingName().contains("Dummy"))
				continue;
			org.mujoco.xml.BodyarchType.Builder<?> linkBody =addBody;
			HashMap<AbstractLink,org.mujoco.xml.BodyarchType.Builder<?>> linkToBulder = new HashMap<>();

			for(int i=0;i<k.getNumberOfLinks();i++) {
				AbstractLink link = k.getAbstractLink(i);
				LinkConfiguration conf = link.getLinkConfiguration();
				ArrayList<CSG>  parts=cadMan.getLinktoCadMap().get(conf);
				if(checkLinkPhysics(cadMan,conf))
					linkBody=loadLink(k,i,parts,linkBody,actuators,linkToBulder);
			}
		}
	}


	public org.mujoco.xml.BodyarchType.Builder<?> loadLink(DHParameterKinematics l,int index,ArrayList<CSG>  cad,org.mujoco.xml.BodyarchType.Builder<?> addBody, Builder<?> actuators,HashMap<AbstractLink,org.mujoco.xml.BodyarchType.Builder<?>> linkToBulderMap) {

		AbstractLink link = l.getAbstractLink(index);
		LinkConfiguration conf = link.getLinkConfiguration();

		String name=null;
		for(String s:mapNameToLink.keySet()) {
			if(mapNameToLink.get(s)==link) {
				name=s;
				break;
			}
		}
		if(name ==null)
			throw new RuntimeException("Link name missing from the map!");

		TransformNR location;
		if(index==0) {
			location=l.getRobotToFiducialTransform().copy();
		}else {
			location=new TransformNR(l.getDhLink(index-1).DhStep(0));
		}
		TransformNR local = new TransformNR(l.getDhLink(index).DhStep(0));
		Transform step = TransformFactory.nrToCSG(local);
		double x = location.getX()/1000.0;

		double y = location.getY()/1000.0;

		double z = location.getZ()/1000.0;
		String quat =location.getRotation().getRotationMatrix2QuaturnionW()+" "+
				location.getRotation().getRotationMatrix2QuaturnionX()+" "+
				location.getRotation().getRotationMatrix2QuaturnionY()+" "+
				location.getRotation().getRotationMatrix2QuaturnionZ();
		TransformNR center = conf.getCenterOfMassFromCentroid();
		String centerString = center.getX()/1000.0+" "+
				center.getY()/1000.0+" "+
				center.getZ()/1000.0+" ";
		org.mujoco.xml.BodyarchType.Builder<?> linkBody = addBody.addBody()
				.withName(name)
				.withPos(x+" "+y+" "+z)
				.withQuat(quat);
		//		linkBody.addInertial()
		//				.withMass(conf.getMassKg())
		//				.withPos(centerString)
		//				.withDiaginertia("1 1 1" )
		linkToBulderMap.put(link, linkBody);
		TransformNR axis;
		if(index==0) {
			axis=l.getRobotToFiducialTransform().copy();
		}else {
			axis=l.forwardOffset(l.getLinkTip(index));
		}
		double gear =260;
		gearRatios.put(link,1.0d*gear);
		double upper = Math.toRadians(link.getMaxEngineeringUnits());
		double lower = Math.toRadians(link.getMinEngineeringUnits());
		String range=lower+" "+upper;
		String ctrlRange=(lower*gear)+" "+(upper*gear);
		double rangeVal=upper-lower;
		JointType.Builder<?> jointBuilder = linkBody.addJoint();
		String pos=0+" "+0+" "+0;
		TransformNR unit = new TransformNR()
				.times(new  TransformNR(0,0,1,new RotationNR()));
		String axisJoint =sig(unit.getX())+" "+sig(unit.getY())+" "+sig(unit.getZ());

		jointBuilder
				.withPos(pos)// the kinematic center
				.withAxis(axisJoint) // rotate about the z axis per dh convention
				.withRange(ctrlRange) // engineering units range
				.withRef(BigDecimal.ZERO) // set the reference position on loading as the links 0 degrees value
				.withType(JointtypeType.HINGE) // hinge type
				.withFrictionloss(BigDecimal.valueOf(0.0001))// experementally determined
				//.withLimited(true)
				//.withDamping(BigDecimal.valueOf(0.00001))
				//.withStiffness(BigDecimal.valueOf(1))
				.withName(name)
		;
		actuators.addPosition()// A position controller to model a servo
				.withKp(BigDecimal.valueOf(0.00004)) // experementally determined value
				//.withForcelimited(true)
				//.withForcerange(range)
				.withCtrlrange(ctrlRange)
				.withName(name)// name the motor
				.withJoint(name)// which joint this motor powers
				.withGear(""+gear) // gear ratio between virtual motor and output
				.withKv(BigDecimal.valueOf(0.0000008)) // damping term experementally determenied
				//.withInheritrange(BigDecimal.valueOf(rangeVal));// sets the range of the control signal to match the limits
		for (int i = 0; i < cad.size(); i++) {
			CSG part = cad.get(i);
			if(!checkForPhysics(part))
				continue;
			Affine cGetManipulator = part.getManipulator();
			if(cGetManipulator!=null) {
				String affineNameMapGet = affineNameMap.get(cGetManipulator);
				if(affineNameMapGet!=null) {

					AbstractLink myLink = mapNameToLink.get(affineNameMapGet);
					DHParameterKinematics k = l;
					CSG transformed = part
							.transformed(
							TransformFactory
							.nrToCSG(
							new TransformNR(
							k.getDhLink(
							getLinkIndex(myLink, k)
							).DhStep(0)
							)
							));
					CSG hull = transformed.hull();
					//					if(myLink!=link)
					//						hull = part.hull();
					transformed.setManipulator(new Affine());
					String geomname=name+" "+i;

					try {
						putCSGInAssets(geomname, hull,true);
						org.mujoco.xml.body.GeomType.Builder<?> geom = linkToBulderMap.get(myLink).addGeom();

						ArrayList<CSG> parts = getMapNameToCSGParts(affineNameMapGet);
						parts.add( transformed);
						setCSGMeshToGeom(geomname, geom);
					} catch (IOException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}else {
					//println "ERROR! "+name+" for part "+part.getName()+" produced no matching affine"
				}
			}
		}
		return linkBody;
	}

}


MobileBase cat =(MobileBase)DeviceManager.getSpecificDevice("Marcos",{
	MobileBase cat = (MobileBase) ScriptingEngine.gitScriptRun(
			"https://github.com/OperationSmallKat/Marcos.git",
			"Marcos.xml");
	cat.connect();
	return cat;
})

ArrayList<MobileBase> bases = new ArrayList<>();
bases.add(cat);
ArrayList<CSG> lifted =new ArrayList<>();
ArrayList<CSG> terrain = new ArrayList<>();

MyManager manager = new MyManager("javaCadTest", bases, lifted, terrain, new File("./physicsTest"));
//manager.setIntegratorType(IntegratorType.RK_4);
manager.setIntegratorType(IntegratorType.IMPLICIT);
manager.setTimestep(0.001);
manager.setCondim(3);
//		manager.generateNewModel();
//		File f = manager.getXMLFile();
//		String s = manager.getXML();
//		System.out.println(s);
//		System.out.println(f.getAbsolutePath());
manager.generateNewModel();// generate model before start counting time
BowlerStudioController.clearCSG();
BowlerStudioController.clearUserNodes();
BowlerStudioController.addObject(manager.getAllCSG(),null );
BowlerStudioController.addObject(MobileBaseCadManager.get(cat).getAllCad(),null );
manager.stepAndWait()
long start = System.currentTimeMillis();
double now = 0;
try {
	while((now=manager.getCurrentSimulationTimeSeconds())<50 && !Thread.interrupted()) {
		if(!manager.stepAndWait()) {
			//println ("Real time broken!");
			//break;
		}else {
			//System.out.println("Time "+now);
		}
		long timeSinceStart = System.currentTimeMillis()-start;
		double sec = ((double)timeSinceStart)/1000.0;
		if((sec-5)>now) {

			println ("Simulation froze and restarted! "+sec+" expected "+now);
			break;
		}
	}
}catch(Throwable t) {
	t.printStackTrace(System.out);
}
manager.close();
System.out.println("Success!");

