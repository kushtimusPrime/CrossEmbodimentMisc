import compas_fab
from compas.robots import Configuration
from compas_fab.backends import PyBulletClient

with PyBulletClient() as client:
    # TODO: Change hardcoded Python file
    urdf_filename = compas_fab.get('../../../../../../kinpy_tests/test_model.urdf')
    robot = client.load_robot(urdf_filename)

    configuration = Configuration.from_revolute_values([0.5,0.5])

    frame_WCF = robot.forward_kinematics(configuration)

    print("Frame in the world coordinate system")
    print(frame_WCF)
