import lcm
import matplotlib.pyplot as plt

from sampling import sample_data
from sampling import region_data

class Region(object):
    def __init__(self, position_x, position_y):
        self.position_x = position_x
        self.position_y = position_y



class SamplingVis(object):
    regions = []
    all_samples = []

    def __init__(self, size_x, size_y):
        self.size_x = size_x
        self.size_y = size_y

    def add_region(self, region):
        self.regions.append(region)
        
    def sampling_node_handler(self, channel, data):
        msg = sample_data.decode(data)
        print("Received message on channel \"%s\"" % channel)
        print("   state_x   = %s" % str(msg.state[0]))
        print("   state_y   = %s" % str(msg.state[1]))
        self.all_samples.append(msg.state)

    def region_handler(self, channel, data):
        msg = region_data.decode(data)
        region = Region(msg.position_x, msg.position_y)
        print("Received message on channel \"%s\"" % channel)
        print("   region position x: " + str(msg.position_x[0]) + " to " + str(msg.position_x[1]))
        print("   region position y: " + str(msg.position_y[0]) + " to " + str(msg.position_y[1]))
        self.regions.append(region)

def main():
    lc = lcm.LCM()
    sample_vis = SamplingVis(100, 100)
    subscription = lc.subscribe("REGION", sample_vis.region_handler)
    subscription = lc.subscribe("SAMPLE", sample_vis.sampling_node_handler)

    
    
    # lc = lcm.LCM()
    # subscription = lc.subscribe("SAMPLE", sample_vis.sampling_node_handler)
    print("OOOOOOOOOOOOOOOOOOOOOOOOOO")
    try:
        while True:
            lc.handle()
    except KeyboardInterrupt:
        print 'Resuming...'
        pass
        # continue
        # print("######")
    

    lc.unsubscribe(subscription)
    

    all_states_x = []
    all_states_y = []
    
    for x in sample_vis.all_samples:
        all_states_x.append(x[0])
        all_states_y.append(x[1])

    plt.plot(all_states_x, all_states_y, 'ro')
    plt.show()

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