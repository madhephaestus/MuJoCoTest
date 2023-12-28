import static org.junit.Assert.fail

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
def model = m.getModel();
def data = m.getData();
System.out.println("Run ModelManager for 10 seconds");
IMujocoController controller =  {d, mL->
	// apply controls https://mujoco.readthedocs.io/en/stable/programming/simulation.html#simulation-loop
	if( mL.nu()==mL.nv() )
		MuJoCoLib.mju_scl(d.ctrl(), d.qvel(), -0.1, mL.nv());
};
m.setController(controller);
while (data.time() < 10) {
	m.step();
	// sleep
	Thread.sleep(m.getTimestepMilliSeconds());
	println "Time: "+data.time()
}
m.close();