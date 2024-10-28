from math import asin, acos, atan, pi, sqrt
from Triceratops_Config import RobotConfiguration, state
import numpy as np
import matplotlib.pyplot as plt

class RobotGait():
    def __init__(self):
        self.config = RobotConfiguration()
        self.h = self.config.pangolin_height
    
    def trot(self, t, x_target, z_target, r1, r3, r2, r4):
        Tf=0.5
        if t < Tf:
            phase_1_swing=self.swing_curve_generate(t, Tf, x_target, z_target, 0, 0, 0)
            phase_1_support=self.support_curve_generate(0.5 + t, Tf, x_target, 0.5, 0)
            # TROT
            leg_1_x = phase_1_swing[0]*r1    ; leg_2_x = phase_1_support[0]*r2    ; leg_3_x = - phase_1_swing[0]*r3    ; leg_4_x = - phase_1_support[0]*r4
            leg_1_y = self.h - phase_1_swing[1]; leg_2_y = self.h - phase_1_support[1]; leg_3_y = self.h - phase_1_swing[1]; leg_4_y = self.h - phase_1_support[1]
            
        else:
            phase_2_swing = self.swing_curve_generate(t - 0.5, Tf, x_target, z_target, 0, 0, 0)
            phase_2_support = self.support_curve_generate(t, Tf, x_target, 0.5, 0)
            # TROT
            leg_1_x = phase_2_support[0]*r1    ; leg_2_x = phase_2_swing[0]*r2    ; leg_3_x = - phase_2_support[0]*r3    ; leg_4_x = - phase_2_swing[0]*r4
            leg_1_y = self.h - phase_2_support[1]; leg_2_y = self.h - phase_2_swing[1]; leg_3_y = self.h - phase_2_support[1]; leg_4_y = self.h - phase_2_swing[1]

        foot_locations = np.array([[ leg_1_x,   leg_2_x,    leg_3_x,    leg_4_x],
                                   [-leg_1_y,  -leg_2_y,   -leg_3_y,   -leg_4_y],
                                   [       0,         0,          0,          0]])
        
        state.foot_locations = foot_locations
        return foot_locations
    
    def support_curve_generate(self, t, Tf, x_past, t_past, zf):
        average=x_past/(1-Tf)
        xf=x_past-average*(t-t_past)
        return xf,zf

    def swing_curve_generate(self, t, Tf, xt, zh, x0, z0, xv0):
        if t >= 0 and t < Tf/4:
            xf = (-4*xv0/Tf)*t*t + xv0*t + x0
            
        if t >= Tf/4 and t < (3*Tf)/4:
            xf = ((-4*Tf*xv0-16*xt+16*x0)*t*t*t)/(Tf*Tf*Tf) + ((7*Tf*xv0+24*xt-24*x0)*t*t)/(Tf*Tf) + ((-15*Tf*xv0-36*xt+36*x0)*t)/(4*Tf) + (9*Tf*xv0+16*xt)/(16)
            
        if t > (3*Tf)/4:
            xf = xt

        if t >= 0 and t < Tf/2:
            zf = (16*z0 - 16*zh)*t*t*t/(Tf*Tf*Tf) + (12*zh - 12*z0)*t*t/(Tf*Tf) + z0
        
        if t >= Tf/2:
            zf = (4*z0 - 4*zh)*t*t/(Tf*Tf) - (4*z0 - 4*zh)*t/(Tf) + z0
            
        x_past = xf
        t_past = t
        
        if zf <= 0:
            zf = 0
        
        return xf, zf, x_past, t_past

# Plotting code
if __name__ == "__main__":
    gait = RobotGait()
    t = 0
    dt = 0.01
    times = []
    leg_positions = {1: [], 2: [], 3: [], 4: []}

    while t <= 1:
        leg_pos = gait.trot(t, 25, 10, 2, 2, 2, 2)
        times.append(t)
        # Store leg positions
        leg_positions[1].append((leg_pos[0, 0], leg_pos[1, 0]))
        leg_positions[2].append((leg_pos[0, 1], leg_pos[1, 1]))
        leg_positions[3].append((leg_pos[0, 2], leg_pos[1, 2]))
        leg_positions[4].append((leg_pos[0, 3], leg_pos[1, 3]))
        t += dt
    
    # Create a figure with 2x2 subplots
    fig, axs = plt.subplots(2, 2, figsize=(12, 8))

    # Plot each leg's position on a separate subplot
    for leg in range(1, 5):
        x_vals = [pos[0] for pos in leg_positions[leg]]
        y_vals = [pos[1] for pos in leg_positions[leg]]
        
        # Determine subplot index (axs[0, 0], axs[0, 1], axs[1, 0], axs[1, 1])
        ax = axs[(leg-1)//2, (leg-1)%2]
        ax.plot(x_vals, y_vals, label=f'Leg {leg}')
        
        # Add titles and labels to each subplot
        ax.set_title(f'Leg {leg} Position during Trot Gait')
        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
        ax.legend()
        ax.grid(True)

    # Adjust layout to prevent overlap
    plt.tight_layout()

    # Show the figure with four subplots
    plt.show()
