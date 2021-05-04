from math import sqrt
import matplotlib.pyplot as plt

DELTA = 0.001


def close_to(x, y):
    if isinstance(x, (tuple, list)) and isinstance(y, (tuple, list)):
        return abs(x[0] - y[0]) <= DELTA and abs(x[1] - y[1]) <= DELTA
    else:
        return abs(x - y) <= DELTA


def get_intersections(circle_0, circle_1):
    x0 = circle_0['x']
    y0 = circle_0['y']
    r0 = circle_0['r']
    x1 = circle_1['x']
    y1 = circle_1['y']
    r1 = circle_1['r']

    # Расстояние между центрами окружностей
    d = sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)

    # Не пересекаются, если расстояние меньше суммы радиусов
    if d > r0 + r1 + 2 * DELTA:
        return None
    # Не пересекаются, если одна окружность внутри другой
    if d < abs(r0 - r1):
        return None
    # Не пересекаются, если идентичные окружности
    if close_to(d, 0) and close_to(r0, r1):
        return None
    else:
        a = (r0 ** 2 - r1 ** 2 + d ** 2) / (2 * d)
        h = sqrt(r0 ** 2 - a ** 2)
        x2 = x0 + a * (x1 - x0) / d
        y2 = y0 + a * (y1 - y0) / d
        x3 = x2 + h * (y1 - y0) / d
        y3 = y2 - h * (x1 - x0) / d

        x4 = x2 - h * (y1 - y0) / d
        y4 = y2 + h * (x1 - x0) / d

        return (x3, y3), (x4, y4)


def get_point(circles_array, to_plot=False):
    goal_point = None
    # Преобразовать массив окружностей в словари
    circles_dict = {}
    for index, coords in enumerate(circles_array):
        circles_dict['c' + str(index)] = dict(x=coords[0], y=coords[1], r=coords[2])

    if to_plot:
        fig, ax = plt.subplots()
        ax.set_xlim((-10, 10))
        ax.set_ylim((-10, 10))
        for circle in iter(circles_dict.values()):
            circle_plot = plt.Circle((circle['x'], circle['y']), circle['r'], fill=False)
            ax.add_artist(circle_plot)
            plt.plot(circle['x'], circle['y'], '.')
    intersections = []
    calculated = []
    for name1, circle1 in iter(circles_dict.items()):
        for name2, circle2 in [[name2, circle2] for name2, circle2 in iter(circles_dict.items()) if circle2 != circle1]:
            if name1 + name2 not in calculated and name2 + name1 not in calculated:
                calculated.append(name1 + name2)
                for xy1, xy2 in get_intersections(circle1, circle2):
                    intersections.append((xy1, xy2))
    n = len(intersections) / 2 - DELTA
    for xy in intersections:
        if len([match for match in intersections if close_to(match, xy)]) > n:
            goal_point = xy
            break
    try:
        print('Point: ', goal_point)
        if to_plot:
            plt.plot(goal_point[0], goal_point[1], 'x')
            plt.gca().set_aspect('equal', adjustable='box')
            plt.show()
    except Exception:
        print('No point!')
    return tuple(map(lambda x: round(x, 4), goal_point))


# intersection circles
circles = [
    [0, 0, 2 * sqrt(2)],
    [0, 2, 2],
    [2, 0, 2],
    # [2, 2, 2]
]

# circles = [[0,0,1], [2,0,1], [1,1,1.001], [1, -1, 1.001], [5,-4, 4*sqrt(2)]]
print(get_point(circles, to_plot=True))
