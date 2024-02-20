import pygame
import env, sensors, Features
import math
import random

def random_color():
    levels = range(32, 256, 32)
    return tuple(random.choice(levels) for _ in range(3))

FeatureMAP = Features.featuresDetection()
environment = env.buildEnvironemnt((600, 1200))
environment.originalMap = environment.map.copy()
laser = sensors.LaserSensor(200, environment.originalMap, uncertainty=(0.5,0.01))
environment.map.fill((0,0,0))
environment.infomap = environment.map.copy()
originalMap = environment.map.copy()
running = True
FEATURE_DETECTION = True
BREAK_POINT_IND = 0

while running:
    environment.infomap = originalMap.copy()
    FEATURE_DETECTION = True
    BREAK_POINT_IND = 0
    ENDPOINTS = [0,0]
    sensorON = False
    PREDICTED_POINTS_TODRAW = []
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    if pygame.mouse.get_focused():
        sensorON = True
    elif not pygame.mouse.get_focused():
        sensorON = False
    if sensorON:
        position=pygame.mouse.get_pos()
        laser.position = position
        sensor_data = laser.sense_obstables()
        FeatureMAP.laser_points_set(sensor_data)
        while BREAK_POINT_IND < (FeatureMAP.NP - FeatureMAP.PMIN):
            COL2 = (0,0,255)
            for element in sensor_data:
                point = environment.AD2Pos(element[0], element[1], element[2])
                # environment.infomap.set_at((int(point[0]), int(point[1])), (0,255,0))
                pygame.draw.circle(environment.infomap, COL2, (int(point[0]), int(point[1])), 2,0)

            seedSeg = FeatureMAP.seed_segment_detection(laser.position, BREAK_POINT_IND)
            if seedSeg == False:
                print("Seedsegs fault")
                break
            else:
                seedSegment = seedSeg[0]
                PREDICTED_POINTS_TODRAW=seedSeg[1]
                INDICES = seedSeg[2]
                results = FeatureMAP.seed_segment_growing(INDICES, BREAK_POINT_IND)
                if results == False:
                    BREAK_POINT_IND = INDICES[1]
                    continue
                else:
                    line_eq = results[1]
                    m,c=results[5]
                    line_seg = results[0]
                    OUTERMOST = results[2]
                    BREAK_POINT_IND = results[3]
                    ENDPOINTS[0] = FeatureMAP.projection_point2line(OUTERMOST[0], m,c)
                    ENDPOINTS[1] = FeatureMAP.projection_point2line(OUTERMOST[1], m, c)
                    COLOR = random_color()
                    for point in line_seg:
                        environment.infomap.set_at((int(point[0][0]), int(point[0][1])), (0,255,0))
                        pygame.draw.circle(environment.infomap, COLOR, (int(point[0][0]), int(point[0][1])), 2,0)

                    pygame.draw.line(environment.infomap, (255, 0,0), ENDPOINTS[0], ENDPOINTS[1], 2)
        print(f'LEN : {len(sensor_data)}')
        print("=" * 20)
            
        environment.dataStorage(sensor_data)
        # environment.show_sensorData()
    environment.map.blit(environment.infomap, (0,0))
    pygame.display.update()