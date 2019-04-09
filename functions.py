import numpy as np
import math
import RobotModel
from RobotModel.cube import Cube
#from collections import Counter

def getMotorsTetha(r: RobotModel, x: float, y: float, z: float):
    """"
    Inverse kinematics function for Uarm
    :param r: RobotModel
    :param x: x position of the destination
    :param y: y position of the destination
    :param z: z position of the destination
    :return: tuple(thetha1,thetha2,thetha3) Rotation of each motor to bring the end effector in the desired position
    """

    arcos = np.arccos
    atan = np.arctan2
    sqrt = np.sqrt
    # arms dimensions and end effector displacement
    a1, a2, a3, disp, thetaDisp = r.getSizes()
    dx, dy, dz = disp
    nx = x
    ny = y
    nz = z + dz
    #print("X:\t", nx, "Y:\t", ny, "Z:\t", nz)
    theta1 = atan(ny, nx)
    r1 = sqrt(ny ** 2 + nx ** 2) + dy
    FOR = 42 * np.pi / 180
    FOL = 25 * np.pi / 180
    s2h2 = r1 ** 2 + nz ** 2
    angleA = arcos((a2 ** 2 + a3 ** 2 - s2h2) / (2 * a2 * a3))
    angleB = np.arctan2(nz, r1)
    angleC = arcos((a2 ** 2 + s2h2 - a3 ** 2) / (2 * a2 * np.sqrt(s2h2)))
    theta3 = np.pi - angleA - angleB - angleC + FOR
    theta2 = angleB + angleC + FOL

    #print("theta1 ", abs(theta1), "| theta2 ", theta2, "| theta3 ", theta3)

    return abs(theta1), theta2, theta3

def roundTheta(angle):
    return

def readVisionData(imageTop, imageFront, rawTop, rawFront):
    blob_countTop = int(imageTop[1][0])
    #print(blob_countTop)

    blob_countFront = int(imageFront[1][0])
    #print(blob_countFront)
    vCntTop = int(imageTop[1][1])
    vCntFront = int(imageFront[1][1])
    blob_info_top = imageTop[1]
    blob_info_front = imageFront[1]

    RAD_2 = np.sqrt(2)

    object_list = {}
    cubes_top = {}
    cubes = []

    # TODO: check
    # TODO: try to use pillow and tesseract to read the letters on the cubes
    for i in range(0, blob_countTop):
        orientation = blob_info_top[3 + (vCntTop * i)]
        posx = blob_info_top[3 + (vCntTop * i) + 1]
        posy = blob_info_top[3 + (vCntTop * i) + 2]
        width = blob_info_top[3 + (vCntTop * i) + 3]
        height = blob_info_top[3 + (vCntTop * i) + 4]

        if width >=0.13:
            if height >= 0.13:
                border_color = getColorName(*readImagesColor(rawTop, posy + height/2 - 1, posx + width/2 - 1))
                if border_color == "grey":#TODO check missing case
                    newx = posx - width/4
                    newy = posy +height/2 - width/4
                    cubes_top.update(assignCoordinateToColor(rawTop,newx,newy))
                    newx = posx + width /4
                    newy = posy - height/2 + width/4
                    cubes_top.update(assignCoordinateToColor(rawTop, newx, newy))
            else:
                if orientation < -0.1:
                    newx = posx - height / (2 * RAD_2)
                    newy = posy + width / (4 * RAD_2)
                    cubes_top.update(assignCoordinateToColor(rawTop, newx, newy))
                    newx = posx + height / (2 * RAD_2)
                    newy = posy - width / (4 * RAD_2)
                    cubes_top.update(assignCoordinateToColor(rawTop, newx, newy))
                else:
                    newx = posx - width/4
                    cubes_top.update(assignCoordinateToColor(rawTop, newx, posy))
                    newx = posx + width / 4
                    cubes_top.update(assignCoordinateToColor(rawTop, newx, posy))
        else:
            if height >= 0.13:
                if orientation < -0.1:
                    newx = posx - width / (2 * RAD_2)
                    newy = posy - height / (4 * RAD_2)
                    cubes_top.update(assignCoordinateToColor(rawTop, newx, newy))
                    newx = posx + width / (2 * RAD_2)
                    newy = posy + height / (4 * RAD_2)
                    cubes_top.update(assignCoordinateToColor(rawTop, newx, newy))

                else:
                    newy = posy + height / 4
                    cubes_top.update(assignCoordinateToColor(rawTop, posx, newy))
                    newy = posy - height / 4
                    cubes_top.update(assignCoordinateToColor(rawTop, posx, newy))
            else:
                cubes_top.update(assignCoordinateToColor(rawTop, posx, posy))
    #print(cubes_top)
    for i in range(0,blob_countFront):
        posx = blob_info_front[3 + (vCntFront * i) + 1]
        posy = blob_info_front[3 + (vCntFront * i) + 2]
        width = blob_info_front[3 + (vCntFront * i) + 3]
        height = blob_info_front[3 + (vCntFront * i) + 4]
        levels = round(height/width)
        if levels == 1:
            color = getColorName(*readImagesColor(rawFront,posy,posx))
            x, y = cubes_top[color]
            object_list.update({color:(x,y,1)})
        else:
            first_block_z = posy+ ((height*(levels-1))/((2*levels)))
            color = getColorName(*readImagesColor(rawFront, first_block_z, posx))
            x, y = cubes_top[color]
            for j in range(0,levels):
                color = getColorName(*readImagesColor(rawFront, first_block_z-((height/levels)*j), posx))
                object_list.update({color: (x, y, levels-j)})
    objList_to_cubeArray(object_list)


    return object_list

def set_index_color(color): #TASK : 432 615
    scheme = {
        "cyan": 1,
        "purple": 2,
        "red": 3,
        "green": 4,
        "bue": 5,
        "yellow": 6
    }
    return scheme.get(color, "grey")

def getMergedVision(listL:dict,listR:dict):
    cubes = {}
    wListL = listL
    wListR = listR

    for key,(x,y,z) in wListL.items():

        if z > 1:
            for keyL,(xc,yc,zc) in listL.items():
                if xc == x and yc == y and zc == z-1 :
                    cubes.update({key:keyL})
        else:
            c = wListR.pop(key,None) # removing the item from right view in order to avoid checking shared cubes in next iteration
            if c != None:
                cubes.update({key: 'shared'})
            else:
                cubes.update({key: 'table1'})

    for key,(x,y,z) in wListR.items():
        if z > 1:
            for keyR,(xc,yc,zc) in listR.items():
                if xc == x and yc == y and zc == z-1 :
                    cubes.update({key:keyR})
        else:
            cubes.update({key: "table2"})

    return cubes


def objList_to_cubeArray(object_list):
    cubes= []
    for obj in dict(object_list):
        c = Cube()
        c.setPosition(*object_list[obj])
        c.set_index_color(obj)
        print("COL: ",obj," POS: ",c.get_x()," | ", c.get_y() ," | ", c.get_z())
        cubes.append(c)


def assignCoordinateToColor(rawImage,x,y):
    return {getColorName(*readImagesColor(rawImage, y, x)):getWorkspaceCoordinates(x, y)}

def readImagesColor(image,y,x): #TODO con np sostuire nell'immagine (0,0,0) con colori vicini
    # TODO pillow
    red, green, blue = image[int(y*128)][int(x*128)][0], image[int(y*128)][int(x*128)][1], image[int(y*128)][int(x*128)][2]
    i = 1
    while red <= 25 and green <= 25 and blue <= 25 :
        red = image[int(y * 128) - i][int(x * 128)][0]
        green = image[int(y * 128) - i][int(x * 128)][1]
        blue = image[int(y * 128) - i][int(x * 128)][2]
        i += 1
    return red, green, blue

def getWorkspaceCoordinates(x: int, y: int):
    """
    Converts blob coordinates

    :param x: X coordinate
    :param y: Y coordinate
    :return: coordinates associated to the workspace
    """
    #return x*2*325 - 325, y*2*325
    return np.interp(x,[0,1],[-530/2,530/2]), np.interp(y,[0,1],[0,530])

def getColorName(r: int, g: int, b: int) -> str:
    #print("r|g|b",r,"|",g,"|",b,"|")

    if r >= 100 and g <= 45 and b <= 45:   return "red"
    if r <= 45 and g >= 100 and b <= 45:   return "green"
    if r <= 45 and g <= 45 and b >= 100:   return "blue"
    if r >= 100 and g <= 45 and b >= 100:  return "purple"
    if r >= 100 and g >=100 and b <= 45:   return "yellow"
    if r <= 45 and g >=100 and b >= 100:   return "cyan"

    return "grey"

def joinEnv(env_left,env_right,arml,armr):
    joined_env = []








def checkMinMaxTetha(theta: float):
    degtheta = np.rad2deg(theta)
    #return (degtheta >= 0 and degtheta <= 180)
    ##print("In deg : ",degtheta,"\n")

