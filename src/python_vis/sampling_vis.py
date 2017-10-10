import lcm
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from sampling import sample_data
from sampling import region_data
from sampling import path_data
from sampling import workspace_size_data

class Region(object):
    def __init__(self, position_x, position_y):
        self.position_x = position_x
        self.position_y = position_y



class SamplingVis(object):
    regions = []
    obstacles = []
    all_samples = []
    workspace_size_x = 0
    workspace_size_y = 0
    # fig = plt.figure(figsize=(10, 10))
    path_x = []
    path_y = []
    # path_x_test = []
    # path_y_test = []



    # def __init__(self, size_x, size_y):
    #     self.size_x = size_x
    #     self.size_y = size_y

    # def add_region(self, region):
    #     self.regions.append(region)

    def workspace_size_handler(self, channel, data):
        msg = workspace_size_data.decode(data)
        print("Received message on channel \"%s\"" % channel)
        print("   workspace x   = %s" % str(msg.size_x))
        print("   workspace y   = %s" % str(msg.size_y))
        self.workspace_size_x = msg.size_x
        self.workspace_size_y = msg.size_y
        
    def sampling_node_handler(self, channel, data):
        msg = sample_data.decode(data)
        print("Received message on channel \"%s\"" % channel)
        print("   state_x   = %s" % str(msg.state[0]))
        print("   state_y   = %s" % str(msg.state[1]))
        self.all_samples.append(msg.state)

    def region_handler(self, channel, data):
        # self.all_samples = []
        msg = region_data.decode(data)
        region = Region(msg.position_x, msg.position_y)
        print("Received message on channel \"%s\"" % channel)
        print("   region position x: " + str(msg.position_x[0]) + " to " + str(msg.position_x[1]))
        print("   region position y: " + str(msg.position_y[0]) + " to " + str(msg.position_y[1]))
        self.regions.append(region)
    
    def obstacle_handler(self, channel, data):
        # self.all_samples = []
        msg = region_data.decode(data)
        region = Region(msg.position_x, msg.position_y)
        print("Received message on channel \"%s\"" % channel)
        print("   region position x: " + str(msg.position_x[0]) + " to " + str(msg.position_x[1]))
        print("   region position y: " + str(msg.position_y[0]) + " to " + str(msg.position_y[1]))
        self.obstacles.append(region)

    def path_handler(self, channel, data):
        # self.all_samples = []
        msg = path_data.decode(data)
        print("Received message on channel \"%s\"" % channel)
        print("Length of path is: " + str(len(msg.state_x)))
        # self.path_x = msg.state_x
        # self.path_y = msg.state_y
        # self.path_x.extend(msg.state_x)
        # self.path_y.extend(msg.state_y)
        self.path_x.append(msg.state_x)
        self.path_y.append(msg.state_y)

    # def path_handler_test(self, channel, data):
    #     msg = path_data.decode(data)
    #     print("Received message on channel \"%s\"" % channel)
    #     print("Length of path is: " + str(len(msg.state_x)))
    #     self.path_x_test = msg.state_x
    #     self.path_y_test = msg.state_y
    
    
    # def region_draw(self, channel, data):
    #     self.fig
    #     plt.axis([0,self.size_x, 0, self.size_y])
    #     rect = patches.Rectangle((50,100),40,30,linewidth=1,edgecolor='r',facecolor='none')
    #     plt.gca().add_patch(rect)
    #     plt.show()

    def samples_draw(self, channel, data):
        all_states_x = []
        all_states_y = []
        for x in self.all_samples:
            all_states_x.append(x[0])
            all_states_y.append(x[1])
        # self.fig
        plt.figure(figsize=(10,10))
        plt.plot(all_states_x, all_states_y, 'ro')

        for rect in self.regions:
            draw_rect = patches.Rectangle((rect.position_x[0],rect.position_y[0]), rect.position_x[1] - rect.position_x[0],rect.position_y[1] - rect.position_y[0],linewidth=1,edgecolor='orange',facecolor='orange')
            currentAxis = plt.gca()
            currentAxis.add_patch(draw_rect)
        
        for rect in self.obstacles:
            draw_rect = patches.Rectangle((rect.position_x[0],rect.position_y[0]), rect.position_x[1] - rect.position_x[0],rect.position_y[1] - rect.position_y[0],linewidth=1,edgecolor='grey',facecolor='grey')
            currentAxis = plt.gca()
            currentAxis.add_patch(draw_rect)

        for x in range(0, len(self.path_x)):
            plt.plot(self.path_x[x], self.path_y[x], color = 'black', linewidth = 2)
        self.path_x = []
        self.path_y = []
        # plt.plot(self.path_x, self.path_y, color = 'black', linewidth = 2)
        # plt.plot(self.path_x_test, self.path_y_test, color = 'blue', linewidth = 2)
        plt.axis([0,self.workspace_size_x, 0, self.workspace_size_y])
        plt.axes().set_aspect('equal')
        
        plt.show()


def main():
    lc = lcm.LCM()
    sample_vis = SamplingVis()
    subscription = lc.subscribe("WORKSPACE", sample_vis.workspace_size_handler)
    # subscription = lc.subscribe("SAMPLE", sample_vis.sampling_node_handler)
    subscription = lc.subscribe("REGION", sample_vis.region_handler)
    subscription = lc.subscribe("OBSTACLE", sample_vis.obstacle_handler)
    subscription = lc.subscribe("SAMPLE", sample_vis.sampling_node_handler)
    subscription = lc.subscribe("PATH", sample_vis.path_handler)
    # subscription = lc.subscribe("PATH_TEST", sample_vis.path_handler_test)
    # subscription = lc.subscribe("DRAW_REGION", sample_vis.region_draw)
    subscription = lc.subscribe("DRAW_SAMPLE", sample_vis.samples_draw)
    
    
    # lc = lcm.LCM()
    # subscription = lc.subscribe("SAMPLE", sample_vis.sampling_node_handler)
    # print("OOOOOOOOOOOOOOOOOOOOOOOOOO")
    try:
        while True:
            lc.handle()
    except KeyboardInterrupt:
        # print 'Resuming...'
        pass
        # continue
        # print("######")
    

    lc.unsubscribe(subscription)
    

    # all_states_x = []
    # all_states_y = []
    
    # for x in sample_vis.all_samples:
    #     all_states_x.append(x[0])
    #     all_states_y.append(x[1])

    # plt.plot(all_states_x, all_states_y, 'ro')
    # plt.show()

if __name__ == '__main__':
    main()
# def my_handler(channel, data):
#     msg = sample_data.decode(data)
#     print("Received message on channel \"%s\"" % channel)
#     print("   state_x   = %s" % str(msg.state[0]))
#     print("   state_y   = %s" % str(msg.state[1]))
#     # plt.plot(msg.state[0], msg.state[1], 'ro', hold=True)
#     # plt.show()
#     # print("   position    = %s" % str(msg.position))
#     # print("   orientation = %s" % str(msg.orientation))
#     # print("   ranges: %s" % str(msg.ranges))
#     # print("   name        = '%s'" % msg.name)
#     # print("   enabled     = %s" % str(msg.enabled))
#     print("")

# # plt.axis([0,100,0,100])
# # plt.hold(True)
# # plt.show()
# lc = lcm.LCM()
# subscription = lc.subscribe("SAMPLE", my_handler)

# try:
#     while True:
#         lc.handle()

# except KeyboardInterrupt:
#     pass

# lc.unsubscribe(subscription)