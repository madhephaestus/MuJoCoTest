import org.mujoco.MuJoCoLib;
import org.mujoco.MuJoCoLib.mjData;
import org.mujoco.MuJoCoLib.mjData_;
import org.mujoco.MuJoCoLib.mjModel;
import org.mujoco.MuJoCoLib.mjModel_;
import org.mujoco.MuJoCoLib.mjVFS;

		System.out.println(System.getProperty("org.bytedeco.javacpp.logger.debug"));
		System.setProperty("org.bytedeco.javacpp.logger.debug", "true");
		MuJoCoLib lib = new MuJoCoLib();

		System.out.println("Starting " + MuJoCoLib.mj_versionString().getString());
		byte[] error = [100] as byte[];
		int error_sz = 0;
		mjModel m = MuJoCoLib.mj_loadXML(
				"/home/hephaestus/git/mujoco-java/src/main/resources/mujoco/java/humanoid/humanoid.xml", 
				(mjVFS)null, 
				error,
				error_sz);
		System.out.println("Humanoid model loaded " + m);
		mjData d = MuJoCoLib.mj_makeData(m);
		try {
			mjModel_ Maccessable = new mjModel_(m);
			mjData_ accessable = new mjData_(d)
				System.out.println("Run model for 10 seconds");
				while (accessable.time() < 10) {
					MuJoCoLib.mj_step(m, d);
					Thread.sleep(1);
					
				}

			
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		System.out.println("Clean up data objects");

		MuJoCoLib.mj_deleteData(d);
		MuJoCoLib.mj_deleteModel(m);