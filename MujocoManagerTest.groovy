
import java.math.BigDecimal

import javax.xml.bind.JAXBException

import org.mujoco.MuJoCoLib
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
import com.neuronrobotics.bowlerstudio.vitamins.Vitamins
import com.neuronrobotics.sdk.addons.kinematics.AbstractLink
import com.neuronrobotics.sdk.addons.kinematics.DHLink
import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.LinkConfiguration
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR
import com.neuronrobotics.sdk.common.DeviceManager
import com.neuronrobotics.sdk.common.IDeviceProvider

import eu.mihosoft.vrl.v3d.CSG
import eu.mihosoft.vrl.v3d.Parabola
import eu.mihosoft.vrl.v3d.RoundedCylinder
import eu.mihosoft.vrl.v3d.Sphere
import eu.mihosoft.vrl.v3d.Transform
import javafx.scene.transform.Affine

public class MyManager extends MuJoCoPhysicsManager{
	
	public MyManager(String name,List<MobileBase> bases, List<CSG> freeObjects, List<CSG> fixedObjects,
		File workingDir) throws IOException, JAXBException {
		super(name, bases, freeObjects, fixedObjects, workingDir);
	}
	public void loadBase(MobileBase cat, org.mujoco.xml.BodyarchType.Builder<?> linkBody2, TransformNR offsetGlobal) throws IOException {
		if(contacts==null)
			contacts= builder.addContact();
		boolean freeBase=cat.getSteerable().size()>0||
				cat.getDrivable().size()>0||
				cat.getLegs().size()>0;
		double KgtoMujocoMass =1.0;
				
		//println "\n\nLowest point "+lowestPoint+" \n\n";
		String bodyName = getMujocoName(cat);
		MobileBaseCadManager cadMan = MobileBaseCadManager.get(cat);
		loadCadForMobileBase(cadMan);
		double lowestPoint = (-computeLowestPoint(cat))/1000.0;
	
		int bodyParts=0;

		org.mujoco.xml.BodyarchType.Builder<?> addBody;
		offsetGlobal=offsetGlobal.times(cat.getRobotToFiducialTransform());
		if(linkBody2==null) {
			addBody= addWorldbody.addBody();
			String centerString = offsetGlobal.getX()/1000.0+" "+
			offsetGlobal.getY()/1000.0+" "+
			lowestPoint;
			String quat =offsetGlobal.getRotation().getRotationMatrix2QuaturnionW()+" "+
						offsetGlobal.getRotation().getRotationMatrix2QuaturnionX()+" "+
						offsetGlobal.getRotation().getRotationMatrix2QuaturnionY()+" "+
						offsetGlobal.getRotation().getRotationMatrix2QuaturnionZ();
			addBody.withPos(centerString)
					.withQuat(quat);// move the base to 1mm above the z=0 surface
			if(freeBase) {
				addBody.addFreejoint();
			}
		}else {
			String centerString = offsetGlobal.getX()/1000.0+" "+
			offsetGlobal.getY()/1000.0+" "+
			offsetGlobal.getZ()/1000.0+" ";
			String quat =offsetGlobal.getRotation().getRotationMatrix2QuaturnionW()+" "+
						offsetGlobal.getRotation().getRotationMatrix2QuaturnionX()+" "+
						offsetGlobal.getRotation().getRotationMatrix2QuaturnionY()+" "+
						offsetGlobal.getRotation().getRotationMatrix2QuaturnionZ();
			addBody=linkBody2.addBody();
			addBody.withPos(centerString)
					.withQuat(quat);// move the base to 1mm above the z=0 surface
			
		}
		addBody.withName(bodyName);
		
		
	//			addBody.addInertial()
	//					.withMass(BigDecimal.valueOf(cat.getMassKg()/1000.0))
	//					.withPos(centerString)
	//					.withDiaginertia("1 1 1" );
		ArrayList<CSG> arrayList = cadMan.getAllCad();
		HashMap<CSG,TransformNR > baseParts=new HashMap<>();
		HashMap<CSG,TransformNR > limbBase=new HashMap<>();

		for (int i = 0; i < arrayList.size(); i++) {
			CSG part = arrayList.get(i);
			if(!checkForPhysics(part))
				continue;
			TransformNR center = cat.getCenterOfMassFromCentroid();
			boolean foundPart = false;
			for(DHParameterKinematics k: cat.getAllDHChains()) {
				if(k.getRootListener() == part.getManipulator()) {
					TransformNR fiducial =k.getRobotToFiducialTransform();
					TransformNR kfed = fiducial;
					//center=kfed.times(center);
					//center=kfed.times(center.inverse());
					center = new TransformNR();
					limbBase.put(part, kfed);
					foundPart=true;
					break;
				}
			}
			if(!foundPart) {
				if(part.getManipulator()!=cat.getRootListener())
					continue;
				limbBase.put(part, new TransformNR());
			}
			baseParts.put(part,center.copy());

		}
		if(baseParts.size()==0){
			CSG CoM =new Sphere(2).toCSG();
			baseParts.put(CoM,cat.getCenterOfMassFromCentroid().copy())
			limbBase.put(CoM, new TransformNR());
		}
		for(CSG part:baseParts.keySet()) {
			TransformNR center = baseParts.get(part);
			TransformNR offset = limbBase.get(part);
			double mass = cat.getMassKg()/baseParts.size();
			bodyParts++;
			String nameOfCSG = bodyName+"_CSG_"+bodyParts;
			
			CSG transformed = part.transformed(TransformFactory.nrToCSG(center.inverse().times(offset)));
			CSG hull ;
			try {
					hull=transformed.hull();
			}catch(Exception ex) {
				hull=transformed;
			}
			transformed=part.transformed(TransformFactory.nrToCSG(offset));
			transformed.setManipulator(new Affine());
			putCSGInAssets(nameOfCSG, hull,true);
			org.mujoco.xml.body.GeomType.Builder<?> geom = addBody.addGeom();
			ArrayList<CSG> parts = getMapNameToCSGParts(bodyName);
			parts.add( transformed);
			setCSGMeshToGeom(nameOfCSG, geom);
			String centerString = center.getX()/1000.0+" "+
					center.getY()/1000.0+" "+
					center.getZ()/1000.0+" ";
			String quat =center.getRotation().getRotationMatrix2QuaturnionW()+" "+
						center.getRotation().getRotationMatrix2QuaturnionX()+" "+
						center.getRotation().getRotationMatrix2QuaturnionY()+" "+
						center.getRotation().getRotationMatrix2QuaturnionZ();
			
			geom.withPos(centerString)
				.withQuat(quat)
				.withMass(BigDecimal.valueOf(mass*KgtoMujocoMass));
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
				String name = conf.getName().trim()+"_"+l.getScriptingName().trim();
				if(!(name.length()>0)) {
					name =""+conf.getXml().hashCode();
				}
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
					linkBody=loadLink(cat,k,i,parts,linkBody,linkToBulder);
				MobileBase follower = k.getFollowerMobileBase(link);
				if(follower!=null) {
					loadBase(follower, linkBody,k.getDHStep(i));
				}
			}
			for(String affineNameMapGet:geomToCSGMap.keySet()) {
				AbstractLink myLink = mapNameToLink.get(affineNameMapGet);
				ArrayList<CSG> parts = getMapNameToCSGParts(affineNameMapGet);
				ArrayList<org.mujoco.xml.body.GeomType.Builder<?>> geoms = geomToCSGMap.get(affineNameMapGet);
				double mass = myLink.getLinkConfiguration().getMassKg()/parts.size();
				if(mass<0.001) {
					mass=0.001;
				}
				for(org.mujoco.xml.body.GeomType.Builder<?>geom:geoms) {
					//println "Mass of "+affineNameMapGet+" is "+mass
					geom.withMass(BigDecimal.valueOf(mass*KgtoMujocoMass));
				}
			}
			
		}
	}
	
	public org.mujoco.xml.BodyarchType.Builder<?> loadLink(
		MobileBase cat,
		DHParameterKinematics l,
		int index,ArrayList<CSG>  cad,
		org.mujoco.xml.BodyarchType.Builder<?> addBody,
		HashMap<AbstractLink,org.mujoco.xml.BodyarchType.Builder<?>> linkToBulderMap) {
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

	double gear =460;
	gearRatios.put(link,1.0d*gear);
	double upper = Math.toRadians(link.getMaxEngineeringUnits());
	double lower = Math.toRadians(link.getMinEngineeringUnits());
	String range=lower+" "+upper;
	String ctrlRange=(lower*gear)+" "+(upper*gear);
	double rangeVal=upper-lower;
	JointType.Builder<?> jointBuilder = linkBody.addJoint();
	String axisJoint ="0 0 1";
	///position=0;
	jointBuilder
			.withPos("0 0 0")// the kinematic center
			.withAxis(axisJoint) // rotate about the z axis per dh convention
			.withRange(ctrlRange) // engineering units range
			.withRef(BigDecimal.valueOf(0)) // set the reference position on loading as the links 0 degrees value
			.withType(JointtypeType.HINGE) // hinge type
			//.withLimited(true)
			//.withDamping(BigDecimal.valueOf(0.00001))
			//.withStiffness(BigDecimal.valueOf(1))
			.withName(name)
	;
	double forceKgCm=3.5;//mg92b default
	Map<String, Object> configELectroMech =Vitamins.getConfiguration(conf.getElectroMechanicalType(), conf.getElectroMechanicalSize());
	Object torque =configELectroMech.get("MaxTorqueNewtonmeters");
	double newtonMeterLimit = 0.0980665*forceKgCm/gear;
	if(torque!=null) {
		newtonMeterLimit=Double.parseDouble(torque.toString())/gear;
	}
	if (!conf.isPassive()) {
		jointBuilder.withFrictionloss(BigDecimal.valueOf(0.01));// experementally determined

		actuators.addPosition()// A position controller to model a servo
				.withKp(BigDecimal.valueOf(0.000006)) // experementally determined value
				//.withForcelimited(true)
				//.withForcerange(range)
				.withCtrlrange(ctrlRange)
				.withForcerange((-newtonMeterLimit)+" "+newtonMeterLimit)
				.withName(name)// name the motor
				.withJoint(name)// which joint this motor powers
				.withGear(""+gear) // gear ratio between virtual motor and output
				.withKv(BigDecimal.valueOf(0.00000001)); // damping term experementally determenied
				//.withInheritrange(BigDecimal.valueOf(rangeVal));// sets the range of the control signal to match the limits
	}
//	CSG CoM  = new Sphere(2)
//					.toCSG()
//					.transformed(
//					TransformFactory
//					.nrToCSG( 
//						cat.getCenterOfMassFromCentroid().copy()
//						)
//					);
//	org.mujoco.xml.body.GeomType.Builder<?> geomCom = linkBody.addGeom();
//	putCSGInAssets(name+"_CoM", CoM,true);
//	setCSGMeshToGeom(name+"_CoM", geomCom);
//	
					
	for (int i = 0; i < cad.size(); i++) {
		CSG part = cad.get(i);
		if(!checkForPhysics(part))
			continue;
		Affine cGetManipulator = part.getManipulator();
		if(cGetManipulator!=null) {
			String affineNameMapGet = affineNameMap.get(cGetManipulator);
			if(affineNameMapGet!=null) {
				DHParameterKinematics k = l;
				
				AbstractLink myLink = mapNameToLink.get(affineNameMapGet);

				double myposition = link.getCurrentEngineeringUnits();
				TransformNR myStep = new TransformNR(
						k.getDhLink(myLink).DhStep(0)
						);
				CSG transformed = part
						.transformed(
						TransformFactory
						.nrToCSG(
						myStep
						))
						;
				CSG hull;
				if(cat.isWheel(myLink)) {
					double height = part.getTotalZ();
					double radius =( part.getTotalY()+part.getTotalX())/4.0;
					double contact =5;
					if(contact>height)
						contact=contact/2;
					CSG cone = Parabola.cone(radius, height/2-contact/2).movez(contact/2);
					cone=cone.union(cone.rotx(180)).hull()
					hull = cone
							.toZMin()
							.movez(transformed.getMinZ())
							;
				}else
					try {
						hull = transformed.hull();
					}catch(Exception ex) {
						hull=transformed;
					}
				//					if(myLink!=link)
				//						hull = part.hull();
				transformed.setManipulator(new Affine());
				String geomname=name+" "+i;

				try {
					putCSGInAssets(geomname, hull,true);
					org.mujoco.xml.body.GeomType.Builder<?> geom = linkToBulderMap.get(myLink).addGeom();

					ArrayList<CSG> parts = getMapNameToCSGParts(affineNameMapGet);
					if(geomToCSGMap.get(affineNameMapGet)==null) {
						geomToCSGMap.put(affineNameMapGet, new ArrayList<org.mujoco.xml.body.GeomType.Builder<?>>());
					}
					geomToCSGMap.get(affineNameMapGet).add(geom);
					parts.add( transformed);
					setCSGMeshToGeom(geomname, geom);
					if(cat.isWheel(myLink)) {
						// default is 1 0.005 0.0001
						//println "Setting Wheel Friction for "+part.getName()
						geom.withFriction("1.7 0.005 0.0001");
					}
					if(cat.isFoot(myLink)) {
						// default is 1 0.005 0.0001
						//println "Setting Foot Friction for "+part.getName()
						geom.withFriction("1.2 0.001 0.00005");
					}
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
//MobileBase cat =(MobileBase)DeviceManager.getSpecificDevice("Standard6dof",{
//	MobileBase cat = (MobileBase) ScriptingEngine.gitScriptRun(
//			"https://github.com/Halloween2020TheChild/GroguMechanicsCad.git",
//			"hephaestus.xml");
//	cat.connect();
//	MobileBaseCadManager.get(cat).setConfigMode(false);
//	MobileBaseCadManager.get(cat).generateCad();
//	return cat;
//})


MobileBase cat =(MobileBase)DeviceManager.getSpecificDevice("NASA_Curiosity",{
	MobileBase cat = (MobileBase) ScriptingEngine.gitScriptRun(
			"https://github.com/NeuronRobotics/NASACurisoity.git",
			"NASA_Curiosity.xml");
	cat.connect();
	MobileBaseCadManager.get(cat).setConfigMode(false);
	MobileBaseCadManager.get(cat).generateCad();
	return cat;
})


//MobileBase cat =(MobileBase)DeviceManager.getSpecificDevice("Marcos",{
//	MobileBase cat = (MobileBase) ScriptingEngine.gitScriptRun(
//			"https://github.com/OperationSmallKat/Marcos.git",
//			"Marcos.xml");
//	cat.connect();
//	MobileBaseCadManager.get(cat).setConfigMode(false);
//	MobileBaseCadManager.get(cat).generateCad();
//	return cat;
//})


ArrayList<MobileBase> bases = new ArrayList<>();
cat.connect();
bases.add(cat);
ArrayList<CSG> lifted =new ArrayList<>();
ArrayList<CSG> terrain = new ArrayList<>();
//List<CSG> parts = (List<CSG>) ScriptingEngine.gitScriptRun(
//	"https://gist.github.com/4814b39ee72e9f590757.git",
//	"javaCad.groovy");
//terrain.add(new Cube(10000,10000,100).toCSG().toZMax());
//for(int i=45;i<parts.size();i++) {
//	if (i==27||i==25)
//		continue;
//	CSG p= parts.get(i);
//	CSG pl=p.roty(15).movez(200);
//	pl.setName(p.getName());
//	lifted.add(pl);
//	terrain.add(p);
//}

MuJoCoPhysicsManager manager = new MyManager("javaCadTest", bases, lifted, terrain, new File("./physicsTest"));
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

def getGetAllCad = MobileBaseCadManager.get(cat).getAllCad()
for(CSG c:getGetAllCad) {
	c.setIsWireFrame(true)
}
BowlerStudioController.addObject(getGetAllCad,null );
for(int i=0;i<1;i++)
manager.stepAndWait()
//return
long start = System.currentTimeMillis();
double now = 0;
try {
	while((now=manager.getCurrentSimulationTimeSeconds())<500 && !Thread.interrupted()) {
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

