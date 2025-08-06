import warnings
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
from numpy import linalg as LA
import pygame
import time

warnings.filterwarnings("ignore", message="Link Base link.*")

# Configure matplotlib
import matplotlib as mpl
mpl.use('Qt5Agg')
for key in ['save', 'fullscreen', 'quit', 'grid', 'home', 'back', 'forward', 'help', 'yscale', 'xscale']:
    mpl.rcParams[f'keymap.{key}'] = []

# Control configuration
joint_mode = False
scale = 0.2
rotation_scale = 1.0  # Separate scale for rotations (5x faster)
dt = 0.05
updating_sliders = False
deadzone = 0.15  # Joystick deadzone

def compute_jacobian_improved(chain, joint_angles):
    frames = chain.forward_kinematics(joint_angles, full_kinematics=True)
    p_e = np.array(frames[-1][:3, 3])
    
    n_actuated = len([link for link in chain.links[1:] if getattr(link, 'bounds', None) != (0, 0)])
    J = np.zeros((6, n_actuated))
    
    col_idx = 0
    for link_idx, link in enumerate(chain.links[1:], 1):
        if getattr(link, 'bounds', None) == (0, 0):
            continue
            
        frame = frames[link_idx]
        p_i = np.array(frame[:3, 3])
        
        if hasattr(link, 'rotation') and link.rotation is not None:
            rot_axis = np.array(link.rotation)
            z_i = frame[:3, :3].dot(rot_axis)
        else:
            z_i = np.array(frame[:3, 2])
        
        Jv = np.cross(z_i, p_e - p_i)
        Jw = z_i
        J[:, col_idx] = np.concatenate((Jv, Jw))
        col_idx += 1
        
    return J

def apply_deadzone(value, deadzone=0.15):
    """Apply deadzone to joystick input"""
    if abs(value) < deadzone:
        return 0.0
    # Scale the remaining range to 0-1
    sign = 1 if value > 0 else -1
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)

# Robot definition
links = [
    OriginLink(),
    URDFLink("base_rotation", [0, 0, 0.1], [0, 0, 0], [0, 0, 1], bounds=(-np.pi, np.pi)),
    URDFLink("shoulder_lift", [0.05, 0, 0], [0, 0, np.pi], [1, 0, 0], bounds=(-np.pi/2, np.pi/2)),
    URDFLink("fixed_joint", [0, 0, 0.2], [0, 0, 0], [0, 0, 1], bounds=(0, 0)),
    URDFLink("joint_4", [0.05, 0, 0], [0, 0, 0], [1, 0, 0], bounds=(-np.pi/2, np.pi/2)),
    URDFLink("joint_5", [0, 0, 0.15], [0, 0, 0], [0, 0, 1], bounds=(-np.pi, np.pi)),
    URDFLink("joint_6", [0.05, 0, 0], [0, 0, 0], [1, 0, 0], bounds=(-np.pi/2, np.pi/2)),
    URDFLink("joint_7", [0, 0, 0.15], [0, 0, 0], [0, 0, 1], bounds=(-np.pi, np.pi))
]

my_chain = Chain(name='spear', links=links)
q = np.zeros(8)
v_cmd = np.zeros(6)

# Initialize pygame for controller input
pygame.init()
pygame.joystick.init()

# Check for controller
if pygame.joystick.get_count() == 0:
    print("No Xbox controller detected! Please connect controller and restart.")
    exit()

controller = pygame.joystick.Joystick(0)
controller.init()
print(f"Controller connected: {controller.get_name()}")

# GUI setup
fig = plt.figure(figsize=(10, 8))
fig.subplots_adjust(left=0.15, right=0.78, bottom=0.1, top=0.9)
ax = fig.add_axes([0.15, 0.3, 0.55, 0.65], projection='3d')
ax_xy = fig.add_axes([0.79, 0.55, 0.18, 0.35])

ax_xy.set_title('XY View')
ax_xy.set_xlim(-0.5, 0.5)
ax_xy.set_ylim(-0.5, 0.5)
ax_xy.set_aspect('equal')
ax_xy.grid(True)

# Add controller instructions
fig.text(0.02, 0.95, "Xbox Controller:", fontsize=12, fontweight='bold')
fig.text(0.02, 0.90, "Left Stick (0,1): Move X/Y", fontsize=10)
fig.text(0.02, 0.86, "Right Stick (2,3): Pitch/Yaw", fontsize=10)
fig.text(0.02, 0.82, "Triggers (4,5): Move Up/Down", fontsize=10)
fig.text(0.02, 0.78, "LB/RB: Roll Left/Right", fontsize=10)
fig.text(0.02, 0.74, "Back: Toggle Mode", fontsize=10)
fig.text(0.02, 0.70, "Start: Reset Pose", fontsize=10)

def update_joint(idx, val):
    global q, updating_sliders
    if updating_sliders:
        return
    q[idx+1] = val
    plot_arm(q)

def update_all_sliders():
    global updating_sliders
    updating_sliders = True
    try:
        for i, slider in enumerate(sliders):
            if i+1 < len(q):
                slider.set_val(q[i+1])
    finally:
        updating_sliders = False

# Create sliders
sliders = []
for i in range(7):
    ax_sl = fig.add_axes([0.02, 0.1 + (6-i)*0.08, 0.12, 0.06])
    slider = Slider(ax_sl, f'J{i+1}', -np.pi, np.pi, valinit=q[i+1])
    slider.on_changed(lambda val, idx=i: update_joint(idx, val))
    sliders.append(slider)

def plot_arm(joint_angles):
    frames = my_chain.forward_kinematics(joint_angles, full_kinematics=True)
    pts = [f[:3, 3] for f in frames]
    X, Y, Z = [p[0] for p in pts], [p[1] for p in pts], [p[2] for p in pts]
    
    ax.clear()
    ax.plot3D(X, Y, Z, linewidth=4, color='gray')
    ax.scatter(X, Y, Z, s=60, color='steelblue', depthshade=True)
    ax.scatter([pts[-1][0]], [pts[-1][1]], [pts[-1][2]], marker='x', color='red', s=80)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim(-0.5, 0.5)
    ax.set_ylim(-0.5, 0.5)
    ax.set_zlim(0.0, 0.5)
    
    xy_pts = np.array(pts)
    ax_xy.clear()
    ax_xy.plot(xy_pts[:, 0], xy_pts[:, 1], '-o', color='gray', linewidth=2, markersize=4)
    ax_xy.scatter(xy_pts[-1, 0], xy_pts[-1, 1], color='red', s=50)
    ax_xy.set_xlim(-0.5, 0.5)
    ax_xy.set_ylim(-0.5, 0.5)
    ax_xy.set_aspect('equal')
    ax_xy.grid(True)
    
    fig.canvas.draw()

def process_controller_input():
    global q, v_cmd, joint_mode
    
    pygame.event.pump()
    
    # Check for mode toggle (Back button)
    if controller.get_button(6):  # Back button
        joint_mode = not joint_mode
        print(f"Mode: {'Joint' if joint_mode else 'Cartesian'}")
        time.sleep(0.3)  # Debounce
    
    # Check for reset (Start button)
    if controller.get_button(7):  # Start button
        q = np.zeros(8)
        plot_arm(q)
        update_all_sliders()
        time.sleep(0.3)  # Debounce
        return
    
    if joint_mode:
        # Joint mode: Use D-pad and buttons for individual joint control
        joint_scale = 0.02
        
        # D-pad for joints 0-3
        if controller.get_hat(0)[0] == -1:  # Left
            q[1] -= joint_scale
        elif controller.get_hat(0)[0] == 1:  # Right
            q[1] += joint_scale
        if controller.get_hat(0)[1] == 1:  # Up
            q[2] += joint_scale
        elif controller.get_hat(0)[1] == -1:  # Down
            q[2] -= joint_scale
            
        # Buttons for other joints
        if controller.get_button(0):  # A
            q[4] += joint_scale
        elif controller.get_button(1):  # B
            q[4] -= joint_scale
        if controller.get_button(2):  # X
            q[5] += joint_scale
        elif controller.get_button(3):  # Y
            q[5] -= joint_scale
            
        plot_arm(q)
        update_all_sliders()
        return
    
    # Cartesian mode: Full 6DOF control
    v_cmd = np.zeros(6)
    
    # Left stick: X/Y movement (axes 0 and 1)
    left_x = apply_deadzone(controller.get_axis(0), deadzone)   # Left/Right
    left_y = apply_deadzone(-controller.get_axis(1), deadzone) # Forward/Back (inverted)
    
    # Right stick: Pitch/Yaw orientation (axes 2 and 3)
    right_x = apply_deadzone(controller.get_axis(2), deadzone)  # Yaw (around Z)
    right_y = apply_deadzone(-controller.get_axis(3), deadzone) # Pitch (around Y, inverted)
    
    # Triggers: Z movement (axes 4 and 5)
    lt = (controller.get_axis(4) + 1) / 2  # Left trigger (0 to 1)
    rt = (controller.get_axis(5) + 1) / 2  # Right trigger (0 to 1)
    z_vel = rt - lt  # Up when RT pressed, down when LT pressed
    
    # Shoulder buttons: Roll
    roll_vel = 0
    if controller.get_button(4):  # Left bumper (roll left)
        roll_vel = -1.0
    elif controller.get_button(5):  # Right bumper (roll right)
        roll_vel = 1.0
    
    # Apply velocities (different scales for position vs rotation)
    v_cmd[0] = left_x * scale           # X velocity
    v_cmd[1] = left_y * scale           # Y velocity  
    v_cmd[2] = z_vel * scale            # Z velocity
    v_cmd[3] = roll_vel * rotation_scale  # Roll (around X) - faster
    v_cmd[4] = right_y * rotation_scale   # Pitch (around Y) - faster
    v_cmd[5] = right_x * rotation_scale   # Yaw (around Z) - faster
    
    # Only compute IK if there's actual input
    if np.any(np.abs(v_cmd) > 0.001):
        J = compute_jacobian_improved(my_chain, q.tolist())
        sv = LA.svd(J, compute_uv=False)
        if sv.min() < 0.01:
            warnings.warn(f"Near singularity: {sv.min():.4f}")
        
        lambda_ = 0.1
        JJT = J.dot(J.T)
        inv = LA.inv(JJT + (lambda_**2) * np.eye(JJT.shape[0]))
        J_pinv = J.T.dot(inv)
        q_dot = J_pinv.dot(v_cmd)
        
        actuated_idx = 0
        for i in range(1, len(q)):
            if i == 3:  # Skip fixed joint
                continue
            if actuated_idx < len(q_dot):
                q[i] += q_dot[actuated_idx] * dt
                actuated_idx += 1
        
        plot_arm(q)
        update_all_sliders()

# Control loop
def update_plot(frame):
    try:
        process_controller_input()
    except pygame.error:
        print("Controller disconnected!")
        return []
    return []

# Start the controller loop
plt.ion()
plot_arm(q)

# Use matplotlib's animation for continuous controller polling
from matplotlib.animation import FuncAnimation
ani = FuncAnimation(fig, update_plot, interval=50, blit=False)  # 20Hz update rate

plt.show(block=True)