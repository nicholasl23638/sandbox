import genesis as gs
from gait_controller import catbot_crawl

def main():
    # 1. Initialize
    gs.init(backend=gs.gpu)

    # 2. Create Scene
    scene = gs.Scene(show_viewer=True)

    # 3. Add entities
    plane = scene.add_entity(gs.morphs.Plane())
    
    # Import the quadruped (Ensure robot.xml is in your project folder)
    robot = scene.add_entity(
        gs.morphs.MJCF(file='robot.xml')
    )

    # 4. Build the scene (Required before querying joint indices)
    scene.build()

    # 5. Query and match joints
    print("\n--- Joint Map ---")

    for joint in robot.joints:
        print(joint.name)
        # name: The string identifier from the XML
        # v_idx: The velocity index in the generalized coordinates
        # print(f"Joint Name: {joint.name: <15} | Index: {joint.v_idx}")

    # 6. Run simulation
    for i in range(1000):
        scene.step()
        angles = catbot_crawl([1, 0], scene.t)

        # angle names = ["FL_leg", "FR_leg", "BL_leg", "BR_leg"] -> angles = [hip, t_a, t_l]

if __name__ == "__main__":
    main()
