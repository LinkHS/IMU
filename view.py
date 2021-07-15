import sys    
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import os

file_dir = os.path.dirname(__file__)
if file_dir not in sys.path:
    sys.path.insert(0, file_dir)

import utilities

def orientation(quats, out_file=None, title_text=None, deltaT=100):
    """Calculates the orienation of an arrow-patch used to visualize a quaternion.
    Uses "_update_func" for the display.   
    """
    
    # Initialize the 3D-figure
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Define the arrow-shape and the top/bottom colors
    delta = 0.01    # "Thickness" of arrow
    corners = [[0, 0, 0.6],
             [0.2, -0.2, 0],
             [0, 0, 0]]
    colors = ['r', 'b']
    
    # Calculate the arrow corners
    corner_array = np.column_stack(corners)
    
    corner_arrays = []
    corner_arrays.append( corner_array + np.r_[0., 0., delta] )
    corner_arrays.append( corner_array - np.r_[0., 0., delta] )
    
    # Calculate the new orientations, given the quaternion orientation
    all_corners = []
    for quat in quats:
        all_corners.append([utilities.rotate_vector(corner_arrays[0], quat), 
                            utilities.rotate_vector(corner_arrays[1], quat)])
        
    # Animate the whole thing, using 'update_func'
    num_frames = len(quats)
    ani = animation.FuncAnimation(fig, _update_orient, num_frames,
                                  fargs=[all_corners, colors, ax, title_text],
                                  interval=deltaT)
    
    # If requested, save the animation to a file
    if out_file is not None:
        try:
            ani.save(out_file)
            print('Orientation animation saved to {0}'.format(out_file))
        except ValueError:
            print('Sorry, no animation saved!')
            print('You probably have to install "ffmpeg", and add it to your PATH.')
    
    plt.show()    
    
    return

def _update_orient(num, all_corners, colors, ax, title=None):
    """For 3D plots it seems to be impossible to only re-set the data values,
    so the plot has to be cleared and re-generated for each frame
    """
    
    # Clear previous plot
    ax.clear()

    # Plot coordinate axes
    ax.plot([-1, 1], [0, 0], [0, 0])
    ax.plot([0, 0], [-1, 1], [0, 0])
    ax.plot([0, 0], [0, 0], [-1, 1])
    #ax.view_init(45,60)
    
    # Format the plot
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim3d(-1,1)
    ax.set_ylim3d(-1,1)
    ax.set_zlim3d(-1,1)
    plt.setp(ax.get_xticklabels(), visible=False)
    plt.setp(ax.get_yticklabels(), visible=False)
    plt.setp(ax.get_zticklabels(), visible=False)

    
    try:
        # Plot and color the top- and bottom-arrow
        for up_down in range(2):
            corners = all_corners[num][up_down]
            ph = ax.plot_trisurf(corners[:,0], corners[:,1], corners[:,2])
            ph.set_color(colors[up_down])
        
        if title is not None:
            plt.title(title)
        
    except RuntimeError:
        # When the triangle is exactly edge-on "plot_trisurf" seems to have a numerical problem
        print('Cannot show triangle edge-on!')
    return

def position(pos):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.scatter(pos[:,0],pos[:,1],pos[:,2],s=1,c='r')
    plt.show()
    
def ani_position(pos, out_file=None, title_text=None, deltaT=1):
    # Initialize the 3D-figure
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    #ax.set_xlim3d(pos[:,0].min(),pos[:,0].max())
    #ax.set_ylim3d(pos[:,1].min(),pos[:,1].max())
    #ax.set_zlim3d(pos[:,2].min(),pos[:,2].max())
    
    # Animate the whole thing, using 'update_func'
    num_frames = len(pos)
    ani = animation.FuncAnimation(fig, _update_pos, num_frames, fargs=[pos, ax, title_text], 
                                  interval=deltaT)
    
    # If requested, save the animation to a file
    if out_file is not None:
        try:
            ani.save(out_file)
            print('Position animation saved to {0}'.format(out_file))
        except ValueError:
            print('Sorry, no animation saved!')
            print('You probably have to install "ffmpeg", and add it to your PATH.')
    
    plt.show()
    
    return

def _update_pos(num, pos, ax, title=None):
    ax.scatter(pos[num,0], pos[num,1], pos[num,2], s=1, c='r')
    return

if __name__ == '__main__':
    
    omega = np.r_[3, 4, 10]     # [rad/s]
    duration = 2
    rate = 100
    q0 = [1, 0, 0, 0]
    out_file = 'demo_patch.mp4'
     
    ## Calculate the orientation
    dt = 1./rate
    num_rep = duration*rate
    omegas = np.tile(omega, [num_rep, 1])
    quaternion = utilities.calc_quat(omegas, q0, dt, 'bf')
    
    orientation(quaternion, out_file, 'Test')
    #viewer = Orientation_OGL(quat_in=quaternion)
    #viewer.run(looping=True, rate=20)

    
    
    
