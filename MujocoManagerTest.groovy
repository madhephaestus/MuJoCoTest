
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
//println cat.getMassKg()
//println cat.getCenterOfMassFromCentroid()
//return


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
List<CSG> parts = (List<CSG>) ScriptingEngine.gitScriptRun(
	"https://gist.github.com/4814b39ee72e9f590757.git",
	"javaCad.groovy");
//terrain.add(new Cube(10000,10000,100).toCSG().toZMax());
for(int i=46;i<parts.size();i++) {
	if (i==27||i==25)
		continue;
	CSG p= parts.get(i);
	CSG pl=p.roty(15).movez(200);
	pl.setName(p.getName());
	lifted.add(pl);
	terrain.add(p);
}
File workingDir = new File(ScriptingEngine.getWorkspace().getAbsolutePath()+"/physics/test/");
MuJoCoPhysicsManager manager = new MyManager("javaCadTest", bases, lifted, terrain, workingDir);
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
//BowlerStudioController.addObject(getGetAllCad,null );
for(int i=0;i<1;i++)
	manager.stepAndWait()
return
long start = System.currentTimeMillis();
double now = 0;
try {
	while((now=manager.getCurrentSimulationTimeSeconds())<1500 && !Thread.interrupted()) {
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

