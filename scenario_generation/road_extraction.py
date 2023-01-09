import json
import math
import xml.etree.ElementTree as ElementTree

import numpy as np


def extrapolateRoadNodes(translated_road_coordinates):
    extrapolated_road_nodes = []
    for road in translated_road_coordinates:
        left_boundary = road['left_boundary']
        right_boundary = road['right_boundary']

        # fit a polynomial to points x and y
        coefficients_left = np.polyfit(left_boundary['x_coordinates'], left_boundary['y_coordinates'], 3)
        coefficients_right = np.polyfit(right_boundary['x_coordinates'], right_boundary['y_coordinates'], 3)

        assert len(left_boundary['x_coordinates']) == len(left_boundary['y_coordinates']) == \
               len(right_boundary['x_coordinates']) == len(right_boundary['y_coordinates'])

        # compute average distance
        distance_left = 0
        distance_right = 0

        for i in range(1, len(left_boundary['x_coordinates'])):
            distance_left += math.sqrt(math.pow(left_boundary['x_coordinates'][i] - left_boundary['x_coordinates'][i - 1], 2) +
                                       math.pow(left_boundary['y_coordinates'][i] - left_boundary['y_coordinates'][i - 1], 2))
            distance_right += math.sqrt(math.pow(right_boundary['x_coordinates'][i] - right_boundary['x_coordinates'][i - 1], 2) +
                                        math.pow(right_boundary['y_coordinates'][i] - right_boundary['y_coordinates'][i - 1], 2))
        distance_left /= len(left_boundary['x_coordinates'])
        distance_right /= len(left_boundary['x_coordinates'])

        # extend coordinates
        extension = 20
        if left_boundary['x_coordinates'][0] > left_boundary['x_coordinates'][-1]:
            new_x_left = np.arange(left_boundary['x_coordinates'][0] + extension,
                                   left_boundary['x_coordinates'][-1] - extension, - distance_left)
        else:
            new_x_left = np.arange(left_boundary['x_coordinates'][0] - extension,
                                   left_boundary['x_coordinates'][-1] + extension, distance_left)

        if right_boundary['x_coordinates'][0] > right_boundary['x_coordinates'][-1]:
            new_x_right = np.arange(right_boundary['x_coordinates'][0] + extension,
                                    right_boundary['x_coordinates'][-1] - extension, - distance_left)
        else:
            new_x_right = np.arange(right_boundary['x_coordinates'][0] - extension,
                                    right_boundary['x_coordinates'][-1] + extension, distance_left)

        # compute new y_coordinates
        new_y_left = np.polyval(coefficients_left, new_x_left)
        new_y_right = np.polyval(coefficients_right, new_x_right)

        new_left = {'x_coordinates': new_x_left, 'y_coordinates': new_y_left}
        new_right = {'x_coordinates': new_x_right, 'y_coordinates': new_y_right}
        extrapolated_road_nodes.append({'left_boundary': new_left, 'right_boundary': new_right})

    return extrapolated_road_nodes


def extractRoadNodes(input_file_path):
    road_nodes = []
    with open(input_file_path) as file:
        data = json.load(file)
        try:
            for r in data['road_nodes']:
                road_nodes.append(r)
        except KeyError:
            print('there are no road nodes in the file')
    return road_nodes


def extractCoordinates(road_nodes):
    road_coordinates = []
    for r in road_nodes:
        road = {}
        x_coordinates = []
        y_coordinates = []
        width = []
        for n in r:
            x_coordinates.append(n[0])
            y_coordinates.append(n[1])
            width.append(n[3])
        road['x_coordinates'] = x_coordinates
        road['y_coordinates'] = y_coordinates
        road['width'] = width
        road_coordinates.append(road)
    return road_coordinates


def translateCoordinates(road_coordinates):
    translated_road_coordinates = []
    for road in road_coordinates:
        first_loop = True
        lane_marking = {'x_coordinates': road['x_coordinates'], 'y_coordinates': road['y_coordinates']}
        left_x_coordinates = []
        left_y_coordinates = []
        right_x_coordinates = []
        right_y_coordinates = []

        for i in range(len(road['x_coordinates']) - 1):
            x1 = road['x_coordinates'][i]
            x2 = road['x_coordinates'][i + 1]
            y1 = road['y_coordinates'][i]
            y2 = road['y_coordinates'][i + 1]

            length = math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))
            offset = road['width'][i] / 2

            x1_left = x1 + offset * (y2 - y1) / length
            x2_left = x2 + offset * (y2 - y1) / length
            y1_left = y1 + offset * (x1 - x2) / length
            y2_left = y2 + offset * (x1 - x2) / length
            x1_right = x1 - offset * (y2 - y1) / length
            x2_right = x2 - offset * (y2 - y1) / length
            y1_right = y1 - offset * (x1 - x2) / length
            y2_right = y2 - offset * (x1 - x2) / length

            if first_loop:
                left_x_coordinates.append(x1_left)
                right_x_coordinates.append(x1_right)
                left_y_coordinates.append(y1_left)
                right_y_coordinates.append(y1_right)
                first_loop = False

            left_x_coordinates.append(x2_left)
            right_x_coordinates.append(x2_right)
            left_y_coordinates.append(y2_left)
            right_y_coordinates.append(y2_right)

        left_boundary = {'x_coordinates': left_x_coordinates, 'y_coordinates': left_y_coordinates}
        right_boundary = {'x_coordinates': right_x_coordinates, 'y_coordinates': right_y_coordinates}
        # 1st lanelet: left to middle
        # 2nd lanelet: middle to right
        translated_road_coordinates.append({'left_boundary': left_boundary, 'right_boundary': lane_marking})
        translated_road_coordinates.append({'left_boundary': lane_marking, 'right_boundary': right_boundary})

    return translated_road_coordinates


def addRoadsToScenario(road_coordinates, empty_scenario, roads_file_path):
    xml_file = ElementTree.parse(empty_scenario)
    lanelet_id = 1

    for road in road_coordinates:
        new_lanelet = ElementTree.SubElement(xml_file.getroot(), 'lanelet')
        new_lanelet.attrib['id'] = str(lanelet_id)
        lanelet_id += 1
        left_bound = ElementTree.SubElement(new_lanelet, 'leftBound')
        right_bound = ElementTree.SubElement(new_lanelet, 'rightBound')

        assert len(road['left_boundary']['x_coordinates']) == len(road['right_boundary']['x_coordinates']) == \
               len(road['left_boundary']['y_coordinates']) == len(road['right_boundary']['y_coordinates'])

        # reverse order to represent correct driving direction
        tmp_length = len(road['left_boundary']['x_coordinates']) - 1

        for k in range(len(road['left_boundary']['x_coordinates'])):
            point_left = ElementTree.SubElement(left_bound, 'point')
            x_left = ElementTree.SubElement(point_left, 'x')
            y_left = ElementTree.SubElement(point_left, 'y')
            # x_left.text = str(road['left_boundary']['x_coordinates'][abs(k - tmp_length)])
            # y_left.text = str(road['left_boundary']['y_coordinates'][abs(k - tmp_length)])
            x_left.text = str(road['left_boundary']['x_coordinates'][k])
            y_left.text = str(road['left_boundary']['y_coordinates'][k])

        for k in range(len(road['right_boundary']['x_coordinates'])):
            point_right = ElementTree.SubElement(right_bound, 'point')
            x_left = ElementTree.SubElement(point_right, 'x')
            y_left = ElementTree.SubElement(point_right, 'y')
            # x_left.text = str(road['right_boundary']['x_coordinates'][abs(k - tmp_length)])
            # y_left.text = str(road['right_boundary']['y_coordinates'][abs(k - tmp_length)])
            x_left.text = str(road['right_boundary']['x_coordinates'][k])
            y_left.text = str(road['right_boundary']['y_coordinates'][k])

        # lanelet_type = ElementTree.SubElement(new_lanelet, 'laneletType')
        # lanelet_type.text = 'highway'

        line_marking = ElementTree.SubElement(left_bound, 'lineMarking')
        line_marking.text = 'solid'
        line_marking = ElementTree.SubElement(right_bound, 'lineMarking')
        line_marking.text = 'solid'

    xml_file.write(roads_file_path)


def extractRoads(input_file_path, empty_scenario, roads_file_path):
    road_nodes = extractRoadNodes(input_file_path)
    road_coordinates = extractCoordinates(road_nodes)
    translated_coordinates = translateCoordinates(road_coordinates)
    extrapolated_road_nodes = extrapolateRoadNodes(translated_coordinates)
    addRoadsToScenario(extrapolated_road_nodes, empty_scenario, roads_file_path)
