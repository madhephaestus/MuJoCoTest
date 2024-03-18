//import static org.junit.Assert.fail

import org.bytedeco.javacpp.BytePointer
import org.bytedeco.javacpp.DoublePointer
import org.bytedeco.javacpp.IntPointer
import org.mujoco.IMujocoController
import org.mujoco.MuJoCoLib;
import org.mujoco.MuJoCoLib.mjData_;
import org.mujoco.MuJoCoLib.mjModel_;
import org.mujoco.MuJoCoLib.mjVFS;
import org.mujoco.MuJoCoModelManager;

import com.neuronrobotics.bowlerstudio.BowlerStudio
import com.neuronrobotics.bowlerstudio.BowlerStudioController
import com.neuronrobotics.bowlerstudio.creature.MobileBaseCadManager
import com.neuronrobotics.bowlerstudio.creature.MobileBaseLoader
import com.neuronrobotics.bowlerstudio.physics.TransformFactory
import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine
import com.neuronrobotics.sdk.addons.kinematics.AbstractLink
import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.LinkConfiguration
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.imu.IMUUpdate
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR
import com.neuronrobotics.sdk.common.DeviceManager

import eu.mihosoft.vrl.v3d.CSG
import eu.mihosoft.vrl.v3d.Cube
import eu.mihosoft.vrl.v3d.Cylinder
import eu.mihosoft.vrl.v3d.Sphere
import eu.mihosoft.vrl.v3d.Transform
import javafx.scene.paint.Color
import javafx.scene.transform.Affine

MobileBase cat;
if(args!=null) {
	cat=args[0]
}else {
	cat = DeviceManager.getSpecificDevice("Marcos", {
		MobileBase m = MobileBaseLoader.fromGit(
				"https://github.com/OperationSmallKat/Marcos.git",
				"Marcos.xml"
				)
		m.connect()
		return m;
	})
}
String bodyName = cat.getScriptingName()+"_base"

Thread.sleep(500);
MobileBaseCadManager cadMan = MobileBaseCadManager.get(cat)
HashMap<String,AbstractLink> linkNameMap = new HashMap<>()
boolean viewer = cadMan.configMode
while(cadMan.getProcesIndictor().get()<0.999) {
	Thread.sleep(100);
	println "Waiting for cad to process "+cadMan.getProcesIndictor().get()
}
Thread.sleep(500);
if(viewer) {
	cadMan.setConfigurationViewerMode(false);
	cadMan.generateCad();
	Thread.sleep(100);
}
while(cadMan.getProcesIndictor().get()<0.999) {
	Thread.sleep(100);
	println "Waiting for cad to process "+cadMan.getProcesIndictor().get()
}

double timestep = 0.005
String muXML = "<mujoco model=\""+cat.getScriptingName()+"\">\n";
muXML+="  <option timestep=\""+timestep+"\"/>\n"
muXML+="  <size njmax=\"8000\" nconmax=\"4000\"/>\n"
muXML+="  <visual>\n"+
		"    <map force=\"0.1\" zfar=\"30\"/>\n"+
		"    <rgba haze=\"0.15 0.25 0.35 1\"/>\n"+
		"  </visual>\n"
muXML+="  <statistic center=\"0 0 0.7\"/>\n"
muXML+="  <asset>\n"+
		"    <texture type=\"skybox\" builtin=\"gradient\" rgb1=\".3 .5 .7\" rgb2=\"0 0 0\" width=\"32\" height=\"512\"/>\n"+
		"    <texture name=\"body\" type=\"cube\" builtin=\"flat\" mark=\"cross\" width=\"128\" height=\"128\" rgb1=\"0.8 0.6 0.4\" rgb2=\"0.8 0.6 0.4\" markrgb=\"1 1 1\" random=\"0.01\"/>\n"+
		"    <material name=\"body\" texture=\"body\" texuniform=\"true\" rgba=\"0.8 0.6 .4 1\"/>\n"+
		"    <texture name=\"grid\" type=\"2d\" builtin=\"checker\" width=\"512\" height=\"512\" rgb1=\".1 .2 .3\" rgb2=\".2 .3 .4\"/>\n"+
		"    <material name=\"grid\" texture=\"grid\" texrepeat=\"1 1\" texuniform=\"true\" reflectance=\".2\"/>\n"+
		"  </asset>\n"

muXML+="  <default>\n"+
		"    <motor ctrlrange=\"-1 1\" ctrllimited=\"true\"/>\n"+
		"    <default class=\"body\">\n"+
		"\n"+
		"      <!-- geoms -->\n"+
		"      <geom type=\"sphere\" condim=\"1\" friction=\".7\" solimp=\".9 .99 .003\" solref=\".015 1\" material=\"body\" group=\"1\"/>\n"+
		"<!-- joints -->\n"+
		"      <joint type=\"hinge\" damping=\"7.5\" stiffness=\"5\" armature=\".01\" limited=\"true\" solimplimit=\"0 .99 .01\"/>\n"
muXML+="    </default>\n"+
		"  </default>\n";

muXML+="  <worldbody>\n"+
		"    <geom name=\"floor\" size=\"0 0 .05\" type=\"plane\" material=\"grid\" condim=\"3\"/>\n"
muXML+=loadBase(cat,linkNameMap,cadMan)
muXML+="\n  </worldbody>\n"

muXML+="    <actuator>\n"
for(String lname:linkNameMap.keySet()) {
	muXML+="      <motor name=\""+lname+"\"       gear=\"120\"  joint=\""+lname+"\"/>\n"
}
muXML+="    </actuator>\n"

muXML+="\n</mujoco>"

String loadBase(MobileBase m,HashMap<String,AbstractLink> map,MobileBaseCadManager cadMan) {
	String name = m.getScriptingName()+"_base"
	ArrayList<CSG>  cad = cadMan.getBasetoCadMap().get(m)

	CSG box =toBox(cad)
	double x = box.getCenterX()/1000.0

	double y = box.getCenterY()/1000.0

	double z = box.getCenterZ()/1000.0
	String XML ="    <body name=\""+name+"\" pos=\""+x+" "+y+" "+z+"\" childclass=\"body\">\n"
	XML+="      <freejoint name=\"root\"/>\n"
	CSG fbox = box.intersect(box.toXMin().movex(box.getCenterX()))
	CSG bbox = box.intersect(box.toXMax().movex(box.getCenterX()))

	//XML+=csgToGeom(name+"_"+0,bbox,true)
	int corners=1

	for(DHParameterKinematics k:m.getAllDHChains()) {
		if(k.getScriptingName().contains("Dummy"))
			continue;
		def toCSGTransformed = new Cube(10).toCSG()
				.transformed(TransformFactory
				.nrToCSG(k.getRobotToFiducialTransform()))
		XML+=csgToGeom(name+"_"+(corners++),toCSGTransformed,true)

		//if(k.getScriptingName().contentEquals("RightFront"))
		XML+=loadLink(k,0,map,cadMan)
	}
	XML+="    </body>\n"
}
CSG toBox(ArrayList<CSG> cad) {
	CSG box = null
	for(CSG c:cad) {
		if(!c.getStorage().getValue("no-physics").isPresent()) {
			if(box==null)
				box=c.getBoundingBox()
			else
				box=box.union(c.getBoundingBox())
		}
	}
	box=box.getBoundingBox()
	return box;
}
String csgToGeom(String name,CSG box, boolean sizeove) {
	double size = box.getTotalY()/1000.0/2
	if(sizeove)
		size=0.01
	double fx=(box.getCenterX()-size/2)/1000.0;
	double fy=(box.getCenterY()-size/2)/1000.0
	double fz=(box.getCenterZ()-size/2)/1000.0

	return "      <geom name=\""+name+"\" pos=\""+fx+" "+fy+" "+fz+"\" size=\""+size+"\"/>\n"
}

int getLinkIndex(AbstractLink l, DHParameterKinematics k) {
	for (int i=0;i<k.getNumberOfLinks();i++) {
		if(k.getAbstractLink(i)==l)
			return i;
	}
	return -1;
}

DHParameterKinematics getLimb(AbstractLink l, MobileBase m) {
	for(DHParameterKinematics k:m.getAllDHChains()) {
		if(getLinkIndex(l, k)>=0)
			return k;
	}
	return null;
}

String loadLink(DHParameterKinematics l,int index,HashMap<String,AbstractLink> map,MobileBaseCadManager cadMan) {
	if(index==l.getNumberOfLinks())
		return ""
	if(index==3)
		return ""
	AbstractLink link = l.getAbstractLink(index)
	LinkConfiguration conf = link.getLinkConfiguration()
	String name = conf.getName()+"_"+l.getScriptingName()
	map.put(name, link)
	println "Adding parts for "+name
	ArrayList<CSG>  cad = cadMan.getLinktoCadMap().get(conf)
	if(cad==null)
		return ""
	CSG box =toBox(cad)
	String spaces=" "
	for(int i=0;i<index;i++) {
		spaces+="  "
	}
	TransformNR location
	if(index==0) {
		location=l.getRobotToFiducialTransform().copy()
	}else {
		location=new TransformNR(l.getDhLink(index-1).DhStep(0))
	}
	TransformNR local = new TransformNR(l.getDhLink(index).DhStep(0))
	//	if(index==0) {
	//		location=l.getRobotToFiducialTransform().copy()
	//	}else {
	//		location=l.forwardOffset(l.getLinkTip(index-1))
	//	}
	//	TransformNR lForwardOffset = l.forwardOffset(l.getLinkTip(index))
	//	TransformNR local = location.inverse().times(lForwardOffset)

	Transform step = TransformFactory.nrToCSG(local)
	double x = location.getX()/1000.0

	double y = location.getY()/1000.0

	double z = location.getZ()/1000.0
	String quat =" quat=\""+location.getRotation().getRotationMatrix2QuaturnionW()+" "+
			location.getRotation().getRotationMatrix2QuaturnionX()+" "+
			location.getRotation().getRotationMatrix2QuaturnionY()+" "+
			location.getRotation().getRotationMatrix2QuaturnionZ()+"\""
	String XML =spaces+"     <body name=\""+name+"\" pos=\""+x+" "+y+" "+z+"\" "+quat+">\n"

	TransformNR axis
	if(index==0) {
		axis=l.getRobotToFiducialTransform().copy()
	}else {
		axis=l.forwardOffset(l.getLinkTip(index))
	}

	String pos = tfToPos(new TransformNR())

	double upper = link.getMaxEngineeringUnits()
	double lower = link.getMinEngineeringUnits()
	String range=lower+" "+upper

	XML+=spaces+"       <joint name=\""+name+"\" "+pos+" "+"range=\""+range+"\" />\n"
	box=new Cube(10).toCSG()
	XML+=spaces+" "+csgToGeom( name+"_A", box.transformed(step),true)
	XML+=spaces+" "+csgToGeom( name+"_B", box,true)

	XML+=loadLink(l,index+1,map,cadMan)

	XML+=spaces+"     </body>\n"
	//println XML
	return XML
}

String tfToPos(TransformNR tf) {
	double x = 0

	double y = 0

	double z = 0
	String STR=" pos=\""+x+" "+y+" "+z+"\""
	TransformNR unit = new TransformNR(0,0,0,tf.inverse().getRotation())
			.times(new  TransformNR(0,0,1,new RotationNR()))
	STR+=" axis=\""+sig(unit.getX())+" "+sig(unit.getY())+" "+sig(unit.getZ())+"\""
	return STR
}

double sig(double x) {
	if(x>0.999)
		return 1;
	if(x<-0.999)
		return -1;
	if(x<0.001 && x>-0.001)
		return 0
	return x;
}

File dir = ScriptingEngine.getRepositoryCloneDirectory(cat.getGitSelfSource()[0])
println "Robot Dir "+dir.getAbsolutePath()
File muFile =  new File(dir.getAbsolutePath()+"/mujoco.xml");
BufferedWriter writer = new BufferedWriter(new FileWriter(muFile.getAbsolutePath()));
writer.write(muXML);
writer.close();

//return null

System.out.println("Loading "+cat.getScriptingName());

//String filename = "model/humanoid/humanoid-ridgid.xml";
//File file = ScriptingEngine.fileFromGit("https://github.com/CommonWealthRobotics/mujoco-java.git", filename)
File file = muFile;
if(!file.exists()) {
	fail("File is missing from the disk");
}


MuJoCoModelManager m = new MuJoCoModelManager(file);

TransformNR convert(DoublePointer cartesianPositions,DoublePointer cartesianQuaturnions, int i,boolean print) {
	DoublePointer coords =cartesianPositions.getPointer(i*3);
	double x = coords.getPointer(0).get()*1000.0;
	double y = coords.getPointer(1).get()*1000.0;
	double z = coords.getPointer(2).get()*1000.0;

	DoublePointer quat =cartesianQuaturnions.getPointer(i*4);
	double qw = quat.getPointer(0).get();
	double qx = quat.getPointer(1).get();
	double qy = quat.getPointer(2).get();
	double qz = quat.getPointer(3).get();
	//if(print)
	//println "coords "+[x,y,z]+" "+[qw, qx, qy, qz]

	RotationNR local = new RotationNR(qw, qx, qy, qz)

	return new TransformNR(x,y,z,local)
}



try {
	def model = m.getModel();
	def data = m.getData();
	System.out.println("Run ModelManager for 10 seconds");
	ArrayList<Double> target = [];
	for(int i=0;i<model.nu();i++) {
		int qposAddr =model.jnt_qposadr().get(i);
		double position = model.qpos0().get(qposAddr);
		target.add(position);
	}
	double sinCounter=0;
	IntPointer acts= model.name_actuatoradr();
	BytePointer modelNames = model.names()
	println modelNames.getString()
	IntPointer intp = model.name_jntadr();
	IntPointer bodyIndex = model.name_bodyadr();
	IntPointer MeshIndex = model.name_meshadr();
	IntPointer GeomIndex = model.name_geomadr();
	DoublePointer geomSize = model.geom_size()
	DoublePointer geomPos = model.geom_pos()
	DoublePointer geomQuat = model.geom_quat()

	IMujocoController controller =  {MuJoCoModelManager managerLocal->
		/**
		 * This illustrates two concepts. First, we are checking 
		 * if the number of controls mjModel.nu equals the number 
		 * of DoFs mjModel.nv. In general, the same callback may 
		 * be used with multiple models depending on how the user
		 *  code is structured, and so it is a good idea to check 
		 *  the model dimensions in the callback. Second, MuJoCo 
		 *  has a library of BLAS-like functions that are very 
		 *  useful; indeed a large part of the code base consists 
		 *  of calling such functions internally. The mju_scl 
		 *  function above scales the velocity vector mjData.qvel 
		 *  by a constant feedback gain and copies the result into 
		 *  the control vector mjData.ctrl.
		 */
		// apply controls https://mujoco.readthedocs.io/en/stable/programming/simulation.html#simulation-loop
		//if( mL.nu()==mL.nv() )MuJoCoLib.mju_scl(d.ctrl(), d.qvel(), -0.5, mL.nv());
		def d = managerLocal.getData();
		def mL = managerLocal.getModel();
		DoublePointer ctrl = d.ctrl();
		DoublePointer pos = d.qpos();
		HashMap<String,AbstractLink> map =  linkNameMap
		HashMap<String ,Double > positions =[]
		for(int i=0;i<mL.njnt();i++) {
			int qposAddr =mL.jnt_qposadr().get(i);
			double position = pos.get(qposAddr);
			BytePointer byp = modelNames.getPointer(intp.getPointer(i).get());
			String name= byp.getString();
			positions.put(name, position)
		}
		double kp =0.03
		for(int i=0;i<mL.nu();i++) {
			//if(i!=1)continue;
			String actName = modelNames.getPointer(acts.getPointer(i).get()).getString()
			double position = Math.toDegrees(positions.get(actName))

			AbstractLink link = map.get(actName)
			if(link==null)
				continue;
			double posTarget = link.getCurrentEngineeringUnits()
			double error = posTarget-position
			double effort = error * kp
			ctrl.put(i, effort);

			//println actName+" "+i+" "+[position,posTarget]
		}
		positions.clear()
		// Update the virtual IMU
		for(int i=0;i<model.nbody();i++) {
			if(bodyName.contentEquals(m.getBodyName(i))) {
				DoublePointer cartesianQuaturnions = data.xquat();
				DoublePointer cartesianPositions = data.xpos();
				TransformNR tf= convert(cartesianPositions,cartesianQuaturnions,i,false)
			
				Double xAcceleration=Math.toDegrees(tf.getRotation().getRotationTilt());
				Double yAcceleration=Math.toDegrees(tf.getRotation().getRotationAzimuth());
				Double zAcceleration=Math.toDegrees(tf.getRotation().getRotationElevation());

				Double rotxAcceleration=0;
				Double rotyAcceleration=0;
				Double rotzAcceleration=0;

				cat.getImu().setVirtualState(new IMUUpdate( xAcceleration, yAcceleration, zAcceleration,
						rotxAcceleration, rotyAcceleration, rotzAcceleration ))
			}
		}
	};
	m.setController(controller);



	for(int i=0;i<model.nu();i++) {
		IntPointer str = intp.getPointer(i);
		BytePointer byp = new BytePointer();
		byp.address = str.get()+modelNames.address;
		println i+" link = "+byp.getString();
	}
	println "Bodys "+model.nbody()
	println "Meshes "+model.nmesh()

	HashMap<Integer,ArrayList<CSG>> map= []
	for(int i=0;i<model.ngeom();i++) {
		int bodyID = model.geom_bodyid().getPointer(i).get();
		IntPointer str = bodyIndex.getPointer(bodyID);
		BytePointer byp = new BytePointer();
		byp = modelNames.getPointer(str.get());
		String bypGetString = byp.getString()
		String geomName = modelNames.getPointer(GeomIndex.getPointer(i).get()).getString()
		println i+" body = "+bypGetString+" geom: "+geomName;

		CSG ball = new Sphere(10).toCSG()
		DoublePointer coords =geomSize.getPointer(i*3);
		double x = coords.getPointer(0).get()*1000.0;
		double y = coords.getPointer(1).get()*1000.0;
		double z = coords.getPointer(2).get()*1000.0;


		int type = model.geom_type().getPointer(i).get()
		/**
		 *
		 mjGEOM_PLANE        = 0,        // plane
		 mjGEOM_HFIELD = 1,                  // height field
		 mjGEOM_SPHERE = 2,                  // sphere
		 mjGEOM_CAPSULE = 3,                 // capsule
		 mjGEOM_ELLIPSOID = 4,               // ellipsoid
		 mjGEOM_CYLINDER = 5,                // cylinder
		 mjGEOM_BOX = 6,                     // box
		 mjGEOM_MESH = 7,                    // mesh
		 */
		switch(type) {
			case MuJoCoLib.mjGEOM_PLANE:
				println "Plane ";
				ball = new Cube(x==0?10000:x,y==0?10000:y,z).toCSG()
						.toZMax()
				ball.setColor(Color.WHITE)
				break;
			case MuJoCoLib.mjGEOM_CAPSULE:
				CSG top = new Sphere(x).toCSG()
				ball = new Cylinder(x,y*2).toCSG()
						.union(top)
						.union(top.movez(y*2))
						.hull()
						.movez(-y)
				break;
			case MuJoCoLib.mjGEOM_SPHERE:
				ball = new Sphere(x).toCSG()
				break;
		}
		TransformNR local = convert(geomPos,geomQuat,i,true)
		Transform nrToCSG = TransformFactory.nrToCSG(local)
		ball=ball.transformed(nrToCSG)
		ball.setManipulator(new Affine())

		ball.setName(bypGetString)
		if(map.get(bodyID)==null)
			map.put(bodyID,[])
		map.get(bodyID).add(ball)
	}
	HashMap<String,AbstractLink> lnmap=linkNameMap

	HashMap<Affine,String> affineNameMap=[]
	for(String s:lnmap.keySet()) {
		Affine a = lnmap.get(s).getGlobalPositionListener()
		//println "Link listener "+s+" is "+a
		affineNameMap.put(a, s)
	}
	for(int i=0;i<model.nbody();i++) {
		String name = m.getBodyName(i);
		if(bodyName.contentEquals(name)) {
			def array = cadMan.getBasetoCadMap().get(cat)
			for(CSG c:array) {
				if(!c.getStorage().getValue("no-physics").isPresent())
					map.get(i).add(c.clone())
			}
		}else {
			AbstractLink link = lnmap.get(name)
			if(link!=null) {
				LinkConfiguration conf = link.getLinkConfiguration()

				def array = cadMan.getLinktoCadMap().get(conf)
				for(CSG c:array) {
					if(!c.getStorage().getValue("no-physics").isPresent()) {
						for(int j=0;j<model.nbody();j++) {
							String jname = m.getBodyName(j);
							Affine cGetManipulator = c.getManipulator()
							if(cGetManipulator!=null) {
								String affineNameMapGet = affineNameMap.get(cGetManipulator)
								println "Checking "+jname+" is the link for part "+c.getName()
								if(affineNameMapGet!=null)
									if(jname.contentEquals(affineNameMapGet)) {
										AbstractLink l = lnmap.get(jname)
										DHParameterKinematics k = getLimb(l, cat)
										int index = getLinkIndex(l, k)
										TransformNR local = new TransformNR(k.getDhLink(index).DhStep(0))
										Transform step = TransformFactory.nrToCSG(local)
										map.get(j).add(c.transformed(step))
									}
							}
						}
					}
				}
			}
		}
	}




	BowlerStudioController.clearCSG()
	for(Integer i:map.keySet()) {
		BowlerStudioController.addObject(map.get(i), null)
	}
	long start = System.currentTimeMillis();
	while (data.time() < 100  && !Thread.interrupted()) {
		long now = System.currentTimeMillis()
		m.step();
		// sleep

		Thread.sleep(m.getTimestepMilliSeconds());
		if(now-start>16) {
			start=now;
			//println "Time: "+data.time()
			DoublePointer cartesianPositions = data.xpos();
			DoublePointer cartesianQuaturnions = data.xquat();
			ArrayList<TransformNR> poss =[]
			for(int i=0;i<model.nbody();i++) {
				poss.add(convert(cartesianPositions,cartesianQuaturnions,i,false))
			}
			BowlerStudio.runLater({

				for(int i=0;i<model.nbody();i++) {

					TransformNR local = poss[i];
					for(CSG bodyBall:map.get(i))
						TransformFactory.nrToAffine(local, bodyBall.getManipulator())
				}
				poss.clear()
			})
		}
	}
}catch(Throwable t) {
	t.printStackTrace(System.out);
}
println "Exiting MuJoCo Simulation"
m.close();



