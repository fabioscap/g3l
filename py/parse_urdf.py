from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import scipy
from scipy.spatial.transform import Rotation as R

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

URDF_PATH = "/home/fabioscap/Desktop/g3l/resources/gen3_lite.urdf"


def xyz_rpy_to_isometry(xyz, rpy):
    """
    Convert xyz + rpy to a 4x4 homogeneous transform (isometry).

    Parameters:
        xyz : (x, y, z)
        rpy : (roll, pitch, yaw) in radians
    """
    x, y, z = xyz

    # Create rotation from roll-pitch-yaw
    rot = R.from_euler("xyz", rpy)  # 'xyz' means roll→pitch→yaw order
    R_mat = rot.as_matrix()

    # Build homogeneous transform
    T = np.eye(4)
    T[:3, :3] = R_mat
    T[:3, 3] = [x, y, z]
    return T


def intersecting_line(p1, d1, p2, d2, tol=1e-6):
    """
    Compute closest points on two lines and the common normal direction.
    Returns: p1_orig, common_normal, closest_point1, closest_point2
    """

    cross = np.cross(d1, d2)
    cross_norm = np.linalg.norm(cross)

    if cross_norm < tol:
        # parallel lines
        r = p2 - p1
        if np.linalg.norm(np.cross(r, d1)) < tol:
            # coincident
            return None, None, None, None
        else:
            # parallel but distinct
            t = np.dot(d1, r) / np.dot(d1, d1)
            pi1 = p1 + t * d1
            pi2 = p2.copy()
            di = d1  # common direction along the line
            return p1.copy(), di, pi1, pi2

    else:
        # skew lines
        A = np.array([[d1.dot(d1), -d1.dot(d2)], [-d1.dot(d2), d2.dot(d2)]])
        b = np.array([d1.dot(p2 - p1), d2.dot(p2 - p1)])
        l1, l2 = np.linalg.solve(A, b)

        pi1 = p1 + l1 * d1
        pi2 = p2 + l2 * d2

        di = pi2 - pi1
        di /= np.linalg.norm(di)
        return p1.copy(), di, pi1, pi2


def dh_row_to_tf(row, q=0.0):
    # assume revolute joints
    a_i, alpha_i, d_i, theta_i = row
    theta_i += q
    cti = np.cos(theta_i)
    sti = np.sin(theta_i)
    cai = np.cos(alpha_i)
    sai = np.sin(alpha_i)

    print(row)

    T = np.array(
        [
            [cti, -cai * sti, sai * sti, a_i * cti],
            [sti, cai * cti, -sai * cti, a_i * sti],
            [0, sai, cai, d_i],
            [0, 0, 0, 1],
        ]
    )
    print(T)
    return T


from matplotlib.animation import FuncAnimation


def plot_arm(dh_table, frame_scale=0.05, interval=50, animate=True):
    """
    Animate a robot arm where each joint continuously rotates.
    dh_table: list of DH rows [a, d, alpha, theta]
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    lines = []
    frame_lines = []

    # Initialize link line
    points = np.zeros((len(dh_table) + 1, 3))
    (line,) = ax.plot(points[:, 0], points[:, 1], points[:, 2], "-o", color="k")
    lines.append(line)

    # Initialize frames
    for _ in range(len(dh_table) + 1):
        (fx,) = ax.plot([0, 0], [0, 0], [0, 0], "r")
        (fy,) = ax.plot([0, 0], [0, 0], [0, 0], "g")
        (fz,) = ax.plot([0, 0], [0, 0], [0, 0], "b")
        frame_lines.append((fx, fy, fz))

    # Set fixed axes limits
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_zlim(0, 2)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Continuous Robot Arm Animation")

    def update(frame_idx):
        # make q rotate continuously (speed depends on frame_idx)
        q = [0.02 * frame_idx * (i + 1) for i in range(len(dh_table))]

        T = np.eye(4)
        points = [T[:3, 3].copy()]
        Ts = [T.copy()]

        for idx, row in enumerate(dh_table):
            if animate:
                T = T @ dh_row_to_tf(row, q[idx])
            else:
                T = T @ dh_row_to_tf(row)
            points.append(T[:3, 3].copy())
            Ts.append(T.copy())
        points = np.array(points)

        # update link line
        lines[0].set_data(points[:, 0], points[:, 1])
        lines[0].set_3d_properties(points[:, 2])

        # update frames
        for i, T in enumerate(Ts):
            origin = T[:3, 3]
            x_axis = origin + frame_scale * T[:3, 0]
            y_axis = origin + frame_scale * T[:3, 1]
            z_axis = origin + frame_scale * T[:3, 2]

            fx, fy, fz = frame_lines[i]
            fx.set_data([origin[0], x_axis[0]], [origin[1], x_axis[1]])
            fx.set_3d_properties([origin[2], x_axis[2]])

            fy.set_data([origin[0], y_axis[0]], [origin[1], y_axis[1]])
            fy.set_3d_properties([origin[2], y_axis[2]])

            fz.set_data([origin[0], z_axis[0]], [origin[1], z_axis[1]])
            fz.set_3d_properties([origin[2], z_axis[2]])

        return lines + [l for triple in frame_lines for l in triple]

    ani = FuncAnimation(fig, update, frames=1000, interval=interval, blit=False)
    plt.show()


def main():
    tree = ET.parse(URDF_PATH)

    root = tree.getroot()

    joints = []

    for child in root:
        if child.tag == "joint":
            name = child.attrib["name"]
            xyz = np.array(
                child.find("origin").attrib["xyz"].split(" "), dtype=np.float32
            )
            rpy = np.array(
                child.find("origin").attrib["rpy"].split(" "), dtype=np.float32
            )

            if name.startswith("joint_"):
                axis = np.array(
                    child.find("axis").attrib["xyz"].split(" "), dtype=np.float32
                )

                joint_data = {
                    "name": name,
                    "tf": xyz_rpy_to_isometry(xyz, rpy),
                    "axis": axis,
                }
                joints.append(joint_data)
            elif name == "tool_frame_joint":
                joint_data = {
                    "name": name,
                    "tf": xyz_rpy_to_isometry(xyz, rpy),
                    "axis": np.array([0, 0, 1]),
                }
                tool_joint = joint_data

    joints.sort(key=lambda j: int(j["name"].split("_")[1]))

    if tool_joint:
        joints.append(tool_joint)

    print(joints)
    return
    dh_table = []

    x_prev = np.array([1, 0, 0])
    z_prev = joints[0]["axis"]
    o_prev = np.array([0, 0, 0])
    for idx in range(len(joints) - 1):
        # ADL 09_DirectKinematics 13
        # o_prev = joints[idx]["tf"][:3, 3]
        z_i = joints[idx + 1]["axis"]
        _, x_i, pi1, o_i = intersecting_line(
            o_prev, z_prev, joints[idx + 1]["tf"][:3, 3], z_i
        )

        a_i = (o_i - pi1).dot(x_i)
        d_i = (pi1 - o_prev).dot(z_i)

        cross = np.cross(z_prev, z_i)
        sin_alpha = np.dot(x_i, cross)
        cos_alpha = np.dot(z_prev, z_i)
        alpha_i = np.arctan2(sin_alpha, cos_alpha)

        cross = np.cross(x_prev, x_i)
        sin_theta = np.dot(z_prev, cross)
        cos_theta = np.dot(x_prev, x_i)
        theta_i = np.arctan2(sin_theta, cos_theta)

        x_prev = x_i
        z_prev = z_i
        o_prev = o_i

        dh_table.append([a_i, alpha_i, d_i, theta_i])

    plot_arm(dh_table, animate=True)


if __name__ == "__main__":
    main()
