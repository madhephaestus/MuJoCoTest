import static org.junit.Assert.fail

import org.bytedeco.javacpp.BytePointer
import org.bytedeco.javacpp.IntPointer
import org.mujoco.IMujocoController
import org.mujoco.MuJoCoLib;
import org.mujoco.MuJoCoLib.mjData_;
import org.mujoco.MuJoCoLib.mjModel_;
import org.mujoco.MuJoCoLib.mjVFS;
import org.mujoco.MuJoCoModelManager;

import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine

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
	
	for(int i=0;i<model.nu();i++) {
		IntPointer str = intp.getPointer(i);
		BytePointer byp = new BytePointer();
		byp.address = str.get()+modelNames.address;
		println i+" link = "+byp.getString();
	}
	
	long start = System.currentTimeMillis();
	while (data.time() < 5  && !Thread.interrupted()) {
		long now = System.currentTimeMillis()
		m.step();
		// sleep
		Thread.sleep(m.getTimestepMilliSeconds());
		if(now-start>100) {
			start=now;
			println "Time: "+data.time()
		}
	}
}catch(Throwable t) {
	t.printStackTrace(System.out);
}
m.close();



