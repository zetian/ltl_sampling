import lcm
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

from sampling_3d import sample_data_3d
from sampling_3d import region_data_3d
from sampling_3d import path_data_3d
from sampling_3d import workspace_size_data_3d

class Region(object):
    def __init__(self, position_x, position_y, position_z):
        self.position_x = position_x
        self.position_y = position_y
        self.position_z = position_z

class SamplingVis(object):
    regions = []
    obstacles = []
    all_samples = []
    workspace_size_x = 0
    workspace_size_y = 0
    workspace_size_z = 0
    path_x = []
    path_y = []
    path_z = []

    def workspace_size_handler(self, channel, data):
        msg = workspace_size_data_3d.decode(data)
        print("Received message on channel \"%s\"" % channel)
        print("   workspace x   = %s" % str(msg.size_x))
        print("   workspace y   = %s" % str(msg.size_y))
        print("   workspace z   = %s" % str(msg.size_z))
        self.workspace_size_x = msg.size_x
        self.workspace_size_y = msg.size_y
        
    def sampling_node_handler(self, channel, data):
        msg = sample_data_3d.decode(data)
        print("Received message on channel \"%s\"" % channel)
        print("   state_x   = %s" % str(msg.state[0]))
        print("   state_y   = %s" % str(msg.state[1]))
        print("   state_z   = %s" % str(msg.state[2]))
        self.all_samples.append(msg.state)

    def region_handler(self, channel, data):
        # self.all_samples = []
        msg = region_data_3d.decode(data)
        region = Region(msg.position_x, msg.position_y, msg.position_x)
        print("Received message on channel \"%s\"" % channel)
        print("   region position x: " + str(msg.position_x[0]) + " to " + str(msg.position_x[1]))
        print("   region position y: " + str(msg.position_y[0]) + " to " + str(msg.position_y[1]))
        print("   region position z: " + str(msg.position_z[0]) + " to " + str(msg.position_z[1]))
        self.regions.append(region)
    
    def obstacle_handler(self, channel, data):
        msg = region_data_3d.decode(data)
        region = Region(msg.position_x, msg.position_y, msg.position_z)
        print("Received message on channel \"%s\"" % channel)
        print("   region position x: " + str(msg.position_x[0]) + " to " + str(msg.position_x[1]))
        print("   region position y: " + str(msg.position_y[0]) + " to " + str(msg.position_y[1]))
        print("   region position z: " + str(msg.position_z[0]) + " to " + str(msg.position_z[1]))
        self.obstacles.append(region)

    def path_handler(self, channel, data):
        msg = path_data_3d.decode(data)
        print("Received message on channel \"%s\"" % channel)
        print("Length of path is: " + str(len(msg.state_x)))
        self.path_x.append(msg.state_x)
        self.path_y.append(msg.state_y)
        self.path_z.append(msg.state_z)

    def samples_draw(self, channel, data):
        all_states_x = []
        all_states_y = []
        all_states_z = []
        for x in self.all_samples:
            all_states_x.append(x[0])
            all_states_y.append(x[1])
            all_states_z.append(x[2])
        # self.fig

        fig = plt.figure()

        ax = fig.gca(projection='3d')
        theta = np.linspace(-4 * np.pi, 4 * np.pi, 100)
        z = np.linspace(-2, 2, 100)
        r = z**2 + 1
        x = r * np.sin(theta)
        y = r * np.cos(theta)
        path_x = np.array(self.path_x)
        path_y = np.array(self.path_y)
        path_z = np.array(self.path_z)
        print(len(path_x))
        print(len(path_y))
        print(len(path_z))
        ax.plot(x, y, z, label='parametric curve')
        ax.legend()

        plt.show()

        # plt.figure(figsize=(10,10))
        # plt.plot(all_states_x, all_states_y, 'ro')

        
        # for rect in self.regions:
        #     draw_rect = patches.Rectangle((rect.position_x[0],rect.position_y[0]), rect.position_x[1] - rect.position_x[0],rect.position_y[1] - rect.position_y[0],linewidth=1,edgecolor='orange',facecolor='orange')
        #     currentAxis = plt.gca()
        #     currentAxis.add_patch(draw_rect)
        
        # for rect in self.obstacles:
        #     draw_rect = patches.Rectangle((rect.position_x[0],rect.position_y[0]), rect.position_x[1] - rect.position_x[0],rect.position_y[1] - rect.position_y[0],linewidth=1,edgecolor='grey',facecolor='grey')
        #     currentAxis = plt.gca()
        #     currentAxis.add_patch(draw_rect)
        
        # for x in range(0, len(self.path_x)):
        #     plt.plot(self.path_x[x], self.path_y[x], color = 'black', linewidth = 2)
        # self.path_x = []
        # self.path_y = []
        
        # # plt.plot(self.path_x, self.path_y, color = 'black', linewidth = 2)
        # # plt.plot(self.path_x_test, self.path_y_test, color = 'blue', linewidth = 2)
        # plt.axis([0,self.workspace_size_x, 0, self.workspace_size_y])
        # plt.axes().set_aspect('equal')
        # plt.show()


def main():
    

    lc = lcm.LCM()
    sample_vis = SamplingVis()
    subscription = lc.subscribe("WORKSPACE", sample_vis.workspace_size_handler)
    subscription = lc.subscribe("REGION", sample_vis.region_handler)
    subscription = lc.subscribe("OBSTACLE", sample_vis.obstacle_handler)
    # subscription = lc.subscribe("SAMPLE", sample_vis.sampling_node_handler)
    subscription = lc.subscribe("PATH", sample_vis.path_handler)
    subscription = lc.subscribe("DRAW_SAMPLE", sample_vis.samples_draw)
    
    try:
        while True:
            lc.handle()
    except KeyboardInterrupt:
        # print 'Resuming...'
        pass
        # continue
        # print("######")
    

    lc.unsubscribe(subscription)
    


if __name__ == '__main__':
    main()
