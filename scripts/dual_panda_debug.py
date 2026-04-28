import numpy as np
import os
from viser_utils import setup_viser_with_robot, add_spheres, add_trajectory, add_box
from pathlib import Path
from scipy.spatial.transform import Rotation as R
import vamp
from fire import Fire
import time



def project_config_to_pose(goal_poses, vamp_module, start_config):
    """
    Goal pose is in the form [[qw, qx, qy, qz, x, y, z] * num_eef].
    Returns a config that satisfies the pose
    """
    arg0 = [[float(x) for x in row] for row in goal_poses]
    constraint = vamp_module.TaskSpaceConstraint(
        goal_poses,
        [[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]] * len(goal_poses),
        [-0.0001] * 6 * len(goal_poses),
        [0.0001] * 6 * len(goal_poses)
    )
    composable_constraint = vamp_module.Composable_TaskSpaceConstraint(constraint)
    projected_config = composable_constraint.projectConfiguration(np.array(start_config), 0, 10.0, 0.5, 50, False)
    return projected_config


def se3_matrix_to_se3_vec(se3_matrix):
    """
    Convert a 4x4 SE3 matrix to a 7D vector of the form [qw, qx, qy, qz, x, y, z]
    """
    rotation_matrix = se3_matrix[:3, :3]
    translation = se3_matrix[:3, 3]
    quat = R.from_matrix(rotation_matrix).as_quat(scalar_first = True)  # returns in (w, x, y, z) format
    return np.concatenate([quat, translation])

def main(
    **kwargs,
):
    (vamp_module, planner_func, plan_settings, simp_settings) = (
        vamp.configure_robot_and_planner_with_kwargs("dual_panda", "crrtc", **kwargs)
    )
    e = vamp.Environment()
    sampler = vamp_module.halton()



    robot_dir = Path(__file__).parents[1] / "resources" / "rlbench_panda"
    server, robot = setup_viser_with_robot(robot_dir, "dualpanda_exported_spherized.urdf")

    floor_grid = server.scene.add_grid(
        name="/floor_grid",
        width=10.0,
        height=10.0,
        plane="xy",
        position=(0.0, 0.0, 0.0),
        cell_color=(200, 200, 200),
        section_color=(140, 140, 140),
    )
    # robot.update_cfg(next_sample)

    tray_vamp_cuboid = vamp.Cuboid(
        [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.1 , 0.18 , 0.03]
    )

    obj_pose_in_eef_frame = np.eye(4)

    z_rotation_matrix = R.from_euler('y', 20, degrees=True).as_matrix()
    x_rotation_matrix = R.from_euler('x', 90, degrees=True).as_matrix()
    obj_pose_in_eef_frame[:3, :3] = z_rotation_matrix @ x_rotation_matrix

    obj_pose_in_eef_frame[2, 3] += 0.15
    attachment = vamp.Attachment(obj_pose_in_eef_frame)
    attachment.add_cuboid(tray_vamp_cuboid)
    e.attach(attachment, 0)


    tray_world_pose_7 = np.array([0.9960,-0.0002,0.0002,-0.0895,0.2579,-0.0825,0.8596])
    tray_world_pose = np.eye(4)
    # tray_world_pose[:3, :3] = R.from_euler('xyz', [-2.6408e-02, +9.1930e-03, +1.9834e+01], degrees=True).as_matrix()
    # tray_world_pose[:3, 3] = [+2.5783e-01, -1.3987e-01, +8.5960e-01]



    tray_world_pose[:3, :3] = R.from_quat(tray_world_pose_7[:4], scalar_first=True).as_matrix()
    tray_world_pose[:3, 3] = tray_world_pose_7[4:]


    sample_button = server.gui.add_button("Sample and Project")

    @sample_button.on_click
    def _(event):

        np.set_printoptions(precision=3, suppress=True, linewidth=200)

        q = sampler.next()
        urdf_q = np.concatenate([q[9:], q[:9]])
        robot.update_cfg(urdf_q)
        fk2 = vamp_module.eefk(q)

        leaf1 = server.scene.add_frame(
            "/righteef",
            wxyz=R.from_matrix(fk2[0][:3, :3]).as_quat(scalar_first=True),
            position=fk2[0][:3, 3]

        )
        leaf2 = server.scene.add_frame(
            "/leftteef",
            wxyz=R.from_matrix(fk2[1][:3, :3]).as_quat(scalar_first=True),
            position=fk2[1][:3, 3]
        )

        attachment.set_ee_pose(fk2[0])
        spheres_centers_for_viz = np.array([sphere.position for sphere in attachment.posed_spheres])
        spheres_radii_for_viz = np.array([sphere.r for sphere in attachment.posed_spheres])

        attachment_sph_groups = add_spheres(
            server, spheres_centers_for_viz, spheres_radii_for_viz, colors=[[0, 255, 255]] * len(attachment.posed_spheres), prefix="attachment"
        )

        tray_pose = tray_world_pose
        # tray_pose = fk2[0] @ obj_pose_in_eef_frame

        cuboid_handle = add_box(
            server,
            position=tray_pose[:3, 3],
            half_extents=[0.2100,0.3600,0.0060],
            orientation=R.from_matrix(tray_pose[:3, :3]).as_quat(scalar_first=True),
            color=[0, 255, 0],
            prefix="tray_cuboid",
            opacity=0.5
        )



    while True:
        continue


if __name__ == '__main__':
    Fire(main)
