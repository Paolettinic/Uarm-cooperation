import numpy as np

def index_to_color(index):
    scheme = [
        0,
        "purple",   # 1
        "cyan",     # 2
        "red",      # 3
        "green",    # 4
        "blue",     # 5
        "yellow"    # 6
    ]
    return scheme[index]


def color_to_index(color):  # TASK : 432 615
    scheme = {
        "purple": 1,
        "cyan": 2,
        "red": 3,
        "green": 4,
        "blue": 5,
        "yellow": 6
    }
    return scheme.get(color, "grey")

def assignCoordinateToColor( rawImage, x, y):
    return {getColorName(*readImagesColor(rawImage, y, x)): getWorkspaceCoordinates(x, y)}

def getWorkspaceCoordinates(x: int, y: int):
    """
    Converts blob coordinates

    :param x: X coordinate
    :param y: Y coordinate
    :return: coordinates associated to the workspace
    """
    # return x*2*325 - 325, y*2*325
    return np.interp(x, [0, 1], [-530 / 2, 530 / 2]), np.interp(y, [0, 1], [0, 530])

def getColorName(r: int, g: int, b: int) -> str:
    if r >= 100 and g <= 45 and b <= 45:   return "red"
    if r <= 45 and g >= 100 and b <= 45:   return "green"
    if r <= 45 and g <= 45 and b >= 100:   return "blue"
    if r >= 100 and g <= 45 and b >= 100:  return "purple"
    if r >= 100 and g >= 100 and b <= 45:   return "yellow"
    if r <= 45 and g >= 100 and b >= 100:   return "cyan"

    return "grey"

def readImagesColor( image, y, x):

    red = image[int(y * 128)][int(x * 128)][0]
    green = image[int(y * 128)][int(x * 128)][1]
    blue = image[int(y * 128)][int(x * 128)][2]
    i = 1
    while red <= 25 and green <= 25 and blue <= 25:
        red = image[int(y * 128) - i][int(x * 128)][0]
        green = image[int(y * 128) - i][int(x * 128)][1]
        blue = image[int(y * 128) - i][int(x * 128)][2]
        i += 1
    return red, green, blue

def getSimplifiedVision(list: dict, arm):
    cubes = {}
    wList = list
    for key, (x,y,z) in wList.items():
        if z > 1:
            for keyl, (xc, yc, zc) in list.items():
                if xc == x and yc == y and zc == z - 1:
                    cubes.update({color_to_index(key): color_to_index(keyl)})
        else:
            cubes.update({color_to_index(key): 0})
    return cubes

def getMergedVision(list1: dict, list2: dict):
    cubes = {}
    wList1 = list1.copy()
    wList2 = list2.copy()

    for key, (x, y, z) in wList1.items():
        if z > 1:
            for key1, (xc, yc, zc) in list1.items():
                if xc == x and yc == y and zc == z - 1:
                    cubes.update({color_to_index(key): color_to_index(key1)})
        else:
            c = wList2.pop(key, None)  # removing the item from right view in order to avoid checking shared cubes in next iteration
            if c is not None:
                print("removed ({},key: {}) from {}".format(c, key, wList2))
                cubes.update({color_to_index(key): 'shared'})
            else:
                cubes.update({color_to_index(key): 'table1'})
    for key, (x, y, z) in wList2.items():
        if z > 1:
            for keyR, (xc, yc, zc) in list2.items():
                if xc == x and yc == y and zc == z - 1:
                    cubes.update({color_to_index(key): color_to_index(keyR)})
        else:
            cubes.update({color_to_index(key): "table2"})
    return cubes
