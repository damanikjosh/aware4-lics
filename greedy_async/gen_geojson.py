from data import agents, search_nodes, enemy_nodes, dest_nodes
import math

def xy2latlon(dx, dy):
    new_lat = 1. * 43.81678595809796 + (dy / 6378000) * (180 / math.pi)
    new_lon = 1. * 28.581366799068558 + (dx / 6378000) * (180 / math.pi) / math.cos(43.81678595809796 * math.pi / 180)

    return new_lon, new_lat


database = dict(search=search_nodes, enemy=enemy_nodes, dest=dest_nodes)

SCALER = 10.
WIDTH = 2.

for name, data in database.items():
    file = open('data/%s.geojson' % (name,), 'w')
    file.write('''{
      "features": [''')

    id = 0
    for node in data:
        if id > 0:
            file.write(',')
        file.write('''
        {
          "type": "Feature",
          "properties": {},
          "geometry": {
            "coordinates": [
              [
                [
                  %.8f,
                  %.8f
                ],''' % xy2latlon(SCALER * (node.location[0] - 50 - WIDTH / 2), SCALER * (node.location[1] - 50 - WIDTH / 2)))
        file.write('''
                [
                  %.8f,
                  %.8f
                ],''' % xy2latlon(SCALER * (node.location[0] - 50 + WIDTH / 2), SCALER * (node.location[1] - 50 - WIDTH / 2)))
        file.write('''
                [
                  %.8f,
                  %.8f
                ],''' % xy2latlon(SCALER * (node.location[0] - 50 + WIDTH / 2), SCALER * (node.location[1] - 50 + WIDTH / 2)))
        file.write('''
                [
                  %.8f,
                  %.8f
                ],''' % xy2latlon(SCALER * (node.location[0] - 50 - WIDTH / 2), SCALER * (node.location[1] - 50 + WIDTH / 2)))
        file.write('''
                [
                  %.8f,
                  %.8f
                ]''' % xy2latlon(SCALER * (node.location[0] - 50 - WIDTH / 2), SCALER * (node.location[1] - 50 - WIDTH / 2)))
        file.write('''
              ]
            ],
            "type": "Polygon"
          },
          "id": "%d"
        }''' % id)
        id += 1

    file.write('''
      ],
      "type": "FeatureCollection"
    }''')
    file.close()