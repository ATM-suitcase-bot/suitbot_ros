import numpy as np
import matplotlib.pyplot as plt #for tmp visualization
import cv2

class PathPerturb:
    #Create an object for calculating path perturbations
    #aka The local planner object
    def __init__(self, world_map, local2world=2, world2real=2):
        #1 is obstacle, 0 free space in the world map
        self.world_map = world_map

        #number of local map pixels in one world map pixel
        self.local2world = 2

        #number of world map pixels in one real world meter
        self.world2real = 2

        self.local2real = local2world*world2real

    #cv2 driven image rotation
    def rotate_image(self, image, angle):
        angle = angle*180/np.pi
        image_center = tuple(np.array(image.shape[1::-1]) / 2)
        rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
        result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
        return result

    #convert real pose (meters) to pixel mode in the fine map
    def get_pix_ind(self, pose):
        x_pix = int(pose[0]*self.local2real)
        y_pix = int(pose[1]*self.local2real)
       
        return [x_pix, y_pix]

    #input the current [x, y, theta] of the robot and its local sensor occupance
    #output a combined global map
    def combo_maps(self, bot_pose, local_map):
        scaled_global = self.world_map.repeat(self.local2world, axis=0).repeat(self.local2world, axis=1)
        rotated_local = self.rotate_image(local_map, bot_pose[2])>0.2

        [x_pix, y_pix] = self.get_pix_ind(bot_pose)
       
        lx = np.shape(rotated_local)[0]
        ly = np.shape(rotated_local)[1]

        combo_sec = np.logical_or(scaled_global[x_pix-lx:x_pix, y_pix-int(ly/2):y_pix+int(ly/2)], rotated_local)
        scaled_global[x_pix-lx:x_pix, y_pix-int(ly/2):y_pix+int(ly/2)] = combo_sec

        return scaled_global

    #get map centered around robot with both world and detected obstacles
    def get_local_combo(self, bot_pose, local_map, dist = 6):
        pix_dis = int(dist*self.world2real*self.local2world)
       
        total_combo = self.combo_maps(bot_pose, local_map)

        [x_pix, y_pix] = self.get_pix_ind(bot_pose)
        local_combo = total_combo[x_pix-pix_dis:x_pix+pix_dis, y_pix-pix_dis:y_pix+pix_dis]
        local_offset = np.array([x_pix, y_pix])-dist*self.local2real

        return [local_combo, local_offset]

    #very simple rotation of point or points about origin, by angle
    def rot_point(self, point, angle):
        x_out = np.cos(angle)*point[0]-np.sin(angle)*point[1]
        y_out = np.sin(angle)*point[0]+np.cos(angle)*point[1]

        return [x_out, y_out]

    #interpolate the points of a pure pursuit path
    def get_pursuit_pts(self, start_pose, end_point, n_interp=50):
        end_in_s = end_point - start_pose[0:2]
        [x_dis, y_dis] = self.rot_point(end_in_s, start_pose[2])
       
        x_pts = np.linspace(0, x_dis, n_interp)
        l = np.hypot(x_dis, y_dis)
        rad = l**2/(2*y_dis)
        y_pts = rad - rad*np.cos(np.arcsin(x_pts/rad))

        [x_back, y_back] = self.rot_point([x_pts, y_pts], -1*start_pose[2])
       
        return [x_back+start_pose[0], y_back+start_pose[1]]

    #check whether a path to a given point will intersect any obstacles
    #bot is attempting to go through the local map from bot_pose, through target, to final
    def check_path(self, local_map, local_offset, bot_pose, target_pt, final_pt):
        local_bot = bot_pos[0:2]*self.local2real - local_offset
        local_target = target_pt*self.local2real - local_offset
       
        [inter_x, inter_y] = self.get_pursuit_pts([local_bot[0], local_bot[1], bot_pose[2]], local_target)
       
        if(np.sum((local_map[(inter_y.astype(int)), (inter_x.astype(int))])) > 0):
            return False

        if(final_pt is None):
            print('first run- no conflict, not checking last pt')
            return True

        local_final = final_pt*self.local2real - local_offset
        new_theta = np.arctan2(inter_y[-2]-inter_y[-1], inter_x[-1]-inter_x[-2])
       
        [inter_x, inter_y] = self.get_pursuit_pts([local_target[0], local_target[1], new_theta], local_final)

        if(np.sum((local_map[(inter_y.astype(int)), (inter_x.astype(int))])) > 0):
            return False

        plt.plot(inter_x, inter_y, color='r')
       
        return True

    #Get a set of potential basic possible motions
    def get_targets(self, bot_pose, t_max=0.6, t_num = 5, d_min = 0.5, d_max = 4.0, d_num=4):
        all_t = np.linspace(-1*t_max, t_max, t_num).repeat(d_num, axis=0)
        all_d = np.tile(np.linspace(d_min, d_max, d_num), t_num)

        x = all_d*np.cos(all_t)
        y = all_d*np.sin(all_t)

        [x, y] = self.rot_point([x, y], -1*bot_pose[2])
        x = x + bot_pose[0]
        y = y + bot_pose[1]

        return [x, y]

    #Evaluate 'how good' a path is- right now just based on how close it gets to a goal
    def get_path_qual(self, new_path, old_goal):
        dist = np.hypot(new_path[0]-old_goal[0], new_path[1]-old_goal[1])
        return dist

    def get_better_path(self, bot_pos, bot_target, combo_map, combo_offset):
        [x_pot, y_pot] = self.get_targets(bot_pos)

        best_target = None
        best_score = 0
        for i in range(len(x_pot)):
            if(self.check_path(combo_map, combo_offset, bot_pos, np.array([x_pot[i], y_pot[i]]), bot_target)):
                path_score = self.get_path_qual(np.array([x_pot[i], y_pot[i]]), bot_target)
               
                if(best_target is None):
                    best_target = np.array([x_pot[i], y_pot[i]])
                    best_score = path_score
                elif(path_score < best_score):
                    best_target = np.array([x_pot[i], y_pot[i]])
                    best_score = path_score
       
        return best_target
       
#parse global map in as boolean array
in_map = np.array(plt.imread("../data/wean_map_grid_edited.jpg"))[:, :, 0] < 50
pp = PathPerturb(in_map)

local_map = np.zeros((20, 20)) #dummy tester local map- in 0.25 m scale
local_map[11:20, 10:12] = 1
bot_pos = np.array([20.0, 24.5, 1.6])
bot_target = np.array([23.0, 19.5])

[combo_map, combo_offset] = pp.get_local_combo(bot_pos, local_map)
path_ok = pp.check_path(combo_map, combo_offset, bot_pos, bot_target, None)

if(not path_ok):
    print('o no, collision detected- replanning')
    best_target = pp.get_better_path(bot_pos, bot_target, combo_map, combo_offset)

    if(best_target is None):
        print('need global replan')
    else:
        print('local plan to {best_target}'+str(best_target))
        plt.imshow(combo_map)
        offset_bot = bot_pos[0:2]*pp.local2real - combo_offset
        offset_goal = bot_target*pp.local2real - combo_offset
        offset_target = best_target*pp.local2real - combo_offset

        plt.quiver(offset_bot[0], offset_bot[1], np.cos(bot_pos[2]), np.sin(bot_pos[2]), color='r')
        plt.scatter(offset_goal[0], offset_goal[1])
        plt.scatter(offset_target[0], offset_target[1])

        full_path_x, full_path_y = pp.get_pursuit_pts([offset_bot[0], offset_bot[1], bot_pos[2]], offset_target)
        plt.plot(full_path_x, full_path_y)
       
        plt.show()

else:
    offset_bot = bot_pos[0:2]*pp.local2real - combo_offset
    offset_goal = bot_target*pp.local2real - combo_offset
    full_path_x, full_path_y = pp.get_pursuit_pts([offset_bot[0], offset_bot[1], bot_pos[2]], offset_goal)

    plt.quiver(offset_bot[0], offset_bot[1], np.cos(bot_pos[2]), np.sin(bot_pos[2]), color='r')
    plt.scatter(offset_goal[0], offset_goal[1])
    plt.plot(full_path_x, full_path_y)
   
    plt.imshow(combo_map)
    plt.show()
    print('no issues with original path')

