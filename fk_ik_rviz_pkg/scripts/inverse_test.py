import compas_fab
from compas.geometry import Frame
from compas_fab.backends import PyBulletClient

with PyBulletClient() as client:
    urdf_filename = compas_fab.get('../../../../../../kinpy_tests/test_model.urdf')
    robot = client.load_robot(urdf_filename)

    # Given 4x4 matrix, it is 4th column (position), first column, and then second column
    frame_WCF = Frame([-0.848,0.200,2.251], [0.462,0,-0.887], [0, 1, 0])
    start_configuration = robot.zero_configuration()

    configuration = robot.inverse_kinematics(frame_WCF, start_configuration)

    print("Found configuration", configuration)
