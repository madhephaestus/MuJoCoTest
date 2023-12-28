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
import eu.mihosoft.vrl.v3d.Sphere
import javafx.scene.transform.Affine

System.out.println("managerTest");
String filename = "model/humanoid/humanoid.xml";
File file = ScriptingEngine.fileFromGit("https://github.com/CommonWealthRobotics/mujoco-java.git", filename)
if(!file.exists()) {
	fail("File is missing from the disk");
}
MuJoCoModelManager m = new MuJoCoModelManager(file);
try {
	def model = m.getModel();
	def data = m.getData();
	System.out.println("Run ModelManager for 10 seconds");
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
		if( mL.nu()==mL.nv() )
			MuJoCoLib.mju_scl(d.ctrl(), d.qvel(), -0.1, mL.nv());
	};
	m.setController(controller);
	
	BytePointer modelNames = model.names()
	println modelNames.getString()
	IntPointer intp = model.name_jntadr();
	IntPointer bodyIndex = model.name_bodyadr();
	
	for(int i=0;i<model.nu();i++) {
		IntPointer str = intp.getPointer(i);
		BytePointer byp = new BytePointer();
		byp.address = str.get()+modelNames.address;
		println i+" link = "+byp.getString();
	}
	ArrayList<CSG> bodyBalls = []
	
	for(int i=0;i<model.nbody();i++) {
		IntPointer str = bodyIndex.getPointer(i);
		BytePointer byp = new BytePointer();
		byp.address = str.get()+modelNames.address;
		String bypGetString = byp.getString()
		
		println i+" body = "+bypGetString;
		CSG ball = new Sphere(10).toCSG()
		ball.setManipulator(new Affine())
		ball.setName(bypGetString)
		bodyBalls.add(ball)
	}
	BowlerStudioController.setCsg(bodyBalls)
	long start = System.currentTimeMillis();
	while (data.time() < 1  && !Thread.interrupted()) {
		long now = System.currentTimeMillis()
		m.step();
		// sleep
		Thread.sleep(m.getTimestepMilliSeconds());
		if(now-start>16) {
			start=now;
			println "Time: "+data.time()
			DoublePointer cartesianPositions = data.xpos();
			DoublePointer cartesianQuaturnions = data.xquat();
			BowlerStudio.runLater({
				
				for(int i=0;i<model.nbody();i++) {
					DoublePointer coords =cartesianPositions.getPointer(i*3);
					double x = coords.getPointer(0).get()*1000.0;
					double y = coords.getPointer(1).get()*1000.0;
					double z = coords.getPointer(2).get()*1000.0;
					
					DoublePointer quat =cartesianQuaturnions.getPointer(i*4);
					double qx = coords.getPointer(0).get();
					double qy = coords.getPointer(1).get();
					double qz = coords.getPointer(2).get();
					double qw = coords.getPointer(3).get();
					
					TransformNR local = new TransformNR(x,y,z,new RotationNR(qw, qx, qy, qz))
					TransformFactory.nrToAffine(local, bodyBalls.get(i).getManipulator())
				}
			})
		}
	}
}catch(Throwable t) {
	t.printStackTrace(System.out);
}
m.close();



