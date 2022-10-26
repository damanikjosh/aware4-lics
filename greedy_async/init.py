from data import agents, search_nodes, enemy_nodes, dest_nodes

import os
curr_path = os.path.dirname(os.path.abspath(__file__))

SCALER = 10.
WIDTH = 2.
DIST = 2.

bases = dict()
means = dict()

file = open(curr_path + '/initial.config', 'w')
for agent in agents:
    if agent.base not in bases:
        bases[agent.base] = []
        means[agent.base] = 0

    bases[agent.base].append(agent)
    means[agent.base] += len(bases[agent.base])

for i in means.keys():
    means[i] = means[i] / len(bases[i]) - 1

for i, base_agents in bases.items():
    for j, agent in enumerate(base_agents):
        if agent.type == 1:
            file.write('iris:')
            file.write('%.2f:%.2f:5:A\n' % (SCALER * (agent.location[0] - 50 + (1. * j - means[i]) * DIST), SCALER * (agent.location[1] - 50)))
        elif agent.type == 3:
            file.write('boat:')
            file.write('%.2f:%.2f:5:A\n' % (SCALER * (agent.location[0] - 50), SCALER * (agent.location[1] - 50 + (1. * j - means[i]) * DIST)))


# for node in enemy_nodes:
#     file.write('boat:%.2f:%.2f:5:B\n' % (node.location[0] * 2 - 100, node.location[1] * 2 - 100))
# file.close()


file = open(curr_path + '/main.world', 'w')
file.write('''<?xml version="1.0"?>
<sdf version="1.5">
  <world name="default">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://smooth</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>
''')

i = 1
for node in search_nodes:
    file.write('''    <include>
      <name>rect red %d</name>
      <uri>model://rect_red</uri>
      <pose>%.2f %.2f 4 0 0 0</pose>
    </include>
''' % (i, SCALER * (node.location[0] - 50), SCALER * (node.location[1] - 50)))
    i += 1

for node in enemy_nodes:
    file.write('''    <include>
      <name>rect blue %d</name>
      <uri>model://rect_blue</uri>
      <pose>%.2f %.2f 0 0 0 0</pose>
    </include>
''' % (i, SCALER * (node.location[0] - 50), SCALER * (node.location[1] - 50)))
    i += 1

for node in dest_nodes:
    file.write('''    <include>
      <name>rect green %d</name>
      <uri>model://rect_green</uri>
      <pose>%.2f %.2f 0 0 0 0</pose>
    </include>
''' % (i, SCALER * (node.location[0] - 50), SCALER * (node.location[1] - 50)))
    i += 1

file.write('''    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>43.81678595809796</latitude_deg>
      <longitude_deg>28.581366799068558</longitude_deg>
      <!-- 43.81678595809796, 28.581366799068558 -->
      <elevation>1.7</elevation>
    </spherical_coordinates>
  </world>
</sdf>
''')
file.close()