
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

MuJoCoPhysicsManager manager = new MuJoCoPhysicsManager("javaCadTest", bases, lifted, terrain, new File("./physicsTest"));
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
