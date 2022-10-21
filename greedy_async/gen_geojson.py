from data import agents, search_nodes, enemy_nodes
import math

def xy2latlon(dx, dy, dz=None):
    new_lat = 1. * 43.81678595809796 + (dy / 6378000) * (180 / math.pi)
    new_lon = 1. * 28.581366799068558 + (dx / 6378000) * (180 / math.pi) / math.cos(43.81678595809796 * math.pi / 180)
    if dz is None:
        return new_lon, new_lat
    else:
        return new_lon, new_lat, float(dz)

file = open('teest.geojson', 'w')
file.write('''{
  "features": [''')

id = 0
for node in search_nodes:
    file.write('''
    {
      "type": "Feature",
      "properties": {},
      "geometry": {
        "coordinates": [
          [
            [
              %.5f,
              %.5f
            ],''' % xy2latlon(node.location[0] * 2 - 100 - 2.5, node.location[1] * 2 - 100 - 2.5))
    file.write('''
            [
              %.5f,
              %.5f
            ],''' % xy2latlon(node.location[0] * 2 - 100 + 2.5, node.location[1] * 2 - 100 - 2.5))
    file.write('''
            [
              %.5f,
              %.5f
            ],''' % xy2latlon(node.location[0] * 2 - 100 + 2.5, node.location[1] * 2 - 100 + 2.5))
    file.write('''
            [
              %.5f,
              %.5f
            ],''' % xy2latlon(node.location[0] * 2 - 100 - 2.5, node.location[1] * 2 - 100 + 2.5))
    file.write('''
            [
              %.5f,
              %.5f
            ]''' % xy2latlon(node.location[0] * 2 - 100 - 2.5, node.location[1] * 2 - 100 - 2.5))
    file.write('''
          ]
        ],
        "type": "Polygon"
      },
      "id": "%d"
    },''' % id)
    id += 1

file.write('''
  ],
  "type": "FeatureCollection"
}''')
file.close()