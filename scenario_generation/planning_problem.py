import math
import xml.etree.ElementTree as ElementTree
from shapely import geometry
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.common.file_writer import OverwriteExistingFile


def createPlanningProblem(file_path, obstacle_id):
    xml_file = ElementTree.parse(file_path)
    root = xml_file.getroot()

    initial_x_position = 0
    initial_y_position = 0
    initial_orientation = 0

    # remove obstacle with id = obstacle_id
    for child in root:
        if child.tag == 'dynamicObstacle':
            if child.attrib['id'] == str(obstacle_id):
                initial_state = child.find('.//initialState')
                # save initial values
                initial_x_position = initial_state.find('.//position').find('.//point').find('.//x').text
                initial_y_position = initial_state.find('.//position').find('.//point').find('.//y').text
                initial_orientation = initial_state.find('.//orientation').find('.//exact').text
                root.remove(child)
                break

    # assume vehicle will follow the lanelet and find lanelet id
    lanelet_id = 3
    for child in root:
        if child.tag == 'lanelet':
            x_coordinates = []
            y_coordinates = []
            for p in child.find('.//leftBound'):
                if p.tag == 'point':
                    x_coordinates.append(float(p.find('.//x').text))
                    y_coordinates.append(float(p.find('.//y').text))
            for p in child.find('.//rightBound'):
                if p.tag == 'point':
                    x_coordinates.insert(0, float(p.find('.//x').text))
                    y_coordinates.insert(0, float(p.find('.//y').text))
            assert len(x_coordinates) == len(y_coordinates)
            coordinates = []
            for i in range(len(x_coordinates)):
                coordinates.append([x_coordinates[i], y_coordinates[i]])
            lanelet = geometry.Polygon(coordinates)
            point = geometry.Point(float(initial_x_position), float(initial_y_position))
            if lanelet.contains(point):
                lanelet_id = child.attrib['id']
                break

    # print(lanelet_id)

    goal_x_position = 0
    goal_y_position = 0
    goal_radius = 0

    # find goal position on lanelet 'lanelet_id'
    for child in root:
        if child.tag == 'lanelet':
            if child.attrib['id'] == str(lanelet_id):
                first_left = child.find('.//leftBound').findall('.//point')[0]
                last_left = child.find('.//leftBound').findall('.//point')[-1]
                first_left_x = float(first_left.find('.//x').text)
                first_left_y = float(first_left.find('.//y').text)
                last_left_x = float(last_left.find('.//x').text)
                last_left_y = float(last_left.find('.//y').text)

                first_right = child.find('.//rightBound').findall('.//point')[0]
                last_right = child.find('.//rightBound').findall('.//point')[-1]
                first_right_x = float(first_right.find('.//x').text)
                first_right_y = float(first_right.find('.//y').text)
                last_right_x = float(last_right.find('.//x').text)
                last_right_y = float(last_right.find('.//y').text)

                # on average, it is enough to compare first with left
                distance_first_left = math.sqrt(math.pow((float(initial_x_position) - first_left_x), 2) +
                                                math.pow((float(initial_y_position) - first_left_y), 2))
                distance_last_left = math.sqrt(math.pow((float(initial_x_position) - last_left_x), 2) +
                                               math.pow((float(initial_y_position) - last_left_y), 2))

                # choose further distance and set goal_radius
                if distance_first_left > distance_last_left:
                    goal_x_position = (first_left_x + first_right_x) / 2
                    goal_y_position = (first_left_y + first_right_y) / 2
                    goal_radius = math.sqrt(math.pow((first_left_x - first_right_x), 2) +
                                            math.pow((first_left_y - first_right_y), 2)) / 2

                else:
                    goal_x_position = (last_left_x + last_right_x) / 2
                    goal_y_position = (last_left_y + last_right_y) / 2
                    goal_radius = math.sqrt(math.pow((last_left_x - last_right_x), 2) +
                                            math.pow((last_left_y - last_right_y), 2)) / 2

                # decrease goal_radius to avoid overlapping
                goal_radius *= 0.9

                # move goal_position towards initial state by goal_radius
                if distance_last_left > distance_last_left:
                    reference = child.find('.//leftBound').findall('.//point')[1]
                else:
                    reference = child.find('.//leftBound').findall('.//point')[-2]

                orientation = math.atan((float(reference.find('.//y').text) - float(first_left_y)) /
                                        (float(reference.find('.//x').text) - float(first_left_x)))
                goal_x_position += goal_radius * math.cos(orientation)
                goal_y_position += goal_radius * math.sin(orientation)

    planning_problem = ElementTree.SubElement(root, 'planningProblem')
    problem_id = obstacle_id
    planning_problem.attrib['id'] = str(problem_id)

    # initial state
    initial_state = ElementTree.SubElement(planning_problem, 'initialState')
    initial_position = ElementTree.SubElement(initial_state, 'position')
    point = ElementTree.SubElement(initial_position, 'point')
    x = ElementTree.SubElement(point, 'x')
    x.text = initial_x_position
    y = ElementTree.SubElement(point, 'y')
    y.text = initial_y_position
    velocity = ElementTree.SubElement(initial_state, 'velocity')
    exact_velocity = ElementTree.SubElement(velocity, 'exact')
    exact_velocity.text = str(20.0)
    orientation = ElementTree.SubElement(initial_state, 'orientation')
    exact_orientation = ElementTree.SubElement(orientation, 'exact')
    exact_orientation.text = initial_orientation
    yaw_rate = ElementTree.SubElement(initial_state, 'yawRate')
    exact_yaw_rate = ElementTree.SubElement(yaw_rate, 'exact')
    exact_yaw_rate.text = str(0)
    slip_angle = ElementTree.SubElement(initial_state, 'slipAngle')
    exact_slip_angle = ElementTree.SubElement(slip_angle, 'exact')
    exact_slip_angle.text = str(0)
    initial_time = ElementTree.SubElement(initial_state, 'time')
    exact_time = ElementTree.SubElement(initial_time, 'exact')
    exact_time.text = str(0)

    # goal state
    goal_state = ElementTree.SubElement(planning_problem, 'goalState')
    goal_position = ElementTree.SubElement(goal_state, 'position')
    circle = ElementTree.SubElement(goal_position, 'circle')
    circle_radius = ElementTree.SubElement(circle, 'radius')
    circle_radius.text = str(goal_radius)
    circle_center = ElementTree.SubElement(circle, 'center')
    x_center = ElementTree.SubElement(circle_center, 'x')
    x_center.text = str(goal_x_position)
    y_center = ElementTree.SubElement(circle_center, 'y')
    y_center.text = str(goal_y_position)
    goal_time = ElementTree.SubElement(goal_state, 'time')
    interval_start = ElementTree.SubElement(goal_time, 'intervalStart')
    interval_start.text = str(0)
    interval_end = ElementTree.SubElement(goal_time, 'intervalEnd')
    interval_end.text = str(1000)
    output_path = 'complete_' + str(obstacle_id) + '_' + file_path
    xml_file.write(output_path)

    # only for formatting
    scenario, planning_problem_set = CommonRoadFileReader(output_path).open()
    fw = CommonRoadFileWriter(scenario, planning_problem_set)
    fw.write_to_file(output_path, OverwriteExistingFile.ALWAYS)
