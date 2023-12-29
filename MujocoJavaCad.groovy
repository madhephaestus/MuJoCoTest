import static org.junit.Assert.fail

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
import com.neuronrobotics.bowlerstudio.physics.TransformFactory
import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR

import eu.mihosoft.vrl.v3d.CSG
import eu.mihosoft.vrl.v3d.Cube
import eu.mihosoft.vrl.v3d.Cylinder
import eu.mihosoft.vrl.v3d.Sphere
import eu.mihosoft.vrl.v3d.Transform
import javafx.scene.paint.Color
import javafx.scene.transform.Affine

System.out.println("managerTest");
String filename = "model/humanoid/humanoid-ridgid.xml";
File file = ScriptingEngine.fileFromGit("https://github.com/CommonWealthRobotics/mujoco-java.git", filename)
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
	IMujocoController controller =  {mjData_ d, mjModel_ mL->
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
		DoublePointer ctrl = d.ctrl();
		DoublePointer pos = d.qpos();
		double gain = 1
		//println "Controls #"+mL.nu()+" positions #"+mL.nq()+" bodys "+mL.nbody()
		int offset = mL.nq()-mL.nu()
		sinCounter+=0.01;
		if(sinCounter>1)
			sinCounter=0;
		int wiggle = 5
		target.set(wiggle,Math.sin(sinCounter*Math.PI*2));
		for(int i=0;i<mL.nu();i++) {
			int qposAddr =mL.jnt_qposadr().get(i);
			double position = pos.get(qposAddr);
			double effort = (target.get(i) -position) * gain; 
			if(effort>1)
				effort=1;
			if(effort<-1)
				effort=-1;
			ctrl.put(i, effort);
			if(i==wiggle)
				println m.getJointName(i)+" "+i+" "+[qposAddr,position,target.get(i),effort]
		}
	};
	m.setController(controller);
	
	BytePointer modelNames = model.names()
	println modelNames.getString()
	IntPointer intp = model.name_jntadr();
	IntPointer bodyIndex = model.name_bodyadr();
	IntPointer MeshIndex = model.name_meshadr();
	IntPointer GeomIndex = model.name_geomadr();
	DoublePointer geomSize = model.geom_size()
	DoublePointer geomPos = model.geom_pos()
	DoublePointer geomQuat = model.geom_quat()
	
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
	
	BowlerStudioController.clearCSG()
	for(Integer i:map.keySet()) {
		BowlerStudioController.addObject(map.get(i), null)
	}
	long start = System.currentTimeMillis();
	while (data.time() < 20  && !Thread.interrupted()) {
		long now = System.currentTimeMillis()
		m.step();
		// sleep
		Thread.sleep(m.getTimestepMilliSeconds());
		if(now-start>16) {
			start=now;
			//println "Time: "+data.time()
			DoublePointer cartesianPositions = data.xpos();
			DoublePointer cartesianQuaturnions = data.xquat();
			BowlerStudio.runLater({
				
				for(int i=0;i<model.nbody();i++) {
					
					TransformNR local = convert(cartesianPositions,cartesianQuaturnions,i,false)
					for(CSG bodyBall:map.get(i))
						TransformFactory.nrToAffine(local, bodyBall.getManipulator())
				}
			})
		}
	}
}catch(Throwable t) {
	t.printStackTrace(System.out);
}
m.close();



