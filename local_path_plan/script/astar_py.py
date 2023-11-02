import numpy as np
import json
import matplotlib.pyplot as plt


class Map:
    def __init__(self):
        # 设置起点，终点所在的行列数，左上角为0，0
        start_row = 17
        start_col = 4
        end_row = 6
        end_col = 13

        self.map = np.loadtxt('my_map222.txt')

        self.map[start_row, start_col] = -100   # 起点
        self.map[end_row, end_col] = 100        # 终点
        # self.map = np.array([[1, 1, 1, 1, 0, 100],
        #                      [1, 0, 0, 1, 0, 1],
        #                      [1, 1, 0, 1, 0, 1],
        #                      [1, -100, 0, 1, 0, 1],
        #                      [1, 1, 1, 1, 1, 1]])

    def get_start_point(self):
        indices = np.where(self.map == -100)
        return indices[0][0], indices[1][0]

    def get_end_point(self):
        indices = np.where(self.map == 100)
        return indices[0][0], indices[1][0]

    def check_grid(self, point):
        return self.map[point.x, point.y]


class Point:
    def __init__(self, x_, y_):
        self.x = x_
        self.y = y_
        self.father = None
        self.G = 0  # 起点到当前节点所花费的消耗
        self.H = 0  # 到终点的预估消耗
        self.F = 0

    def get_x_y(self):
        return self.x, self.y

    def set_GHF(self, G, H, F):
        self.H = H
        self.G = G
        self.F = F


class Astar:
    def __init__(self):
        self.openlist = []
        self.closelist = []
        self.map = Map()
        self.start_x, self.start_y = self.map.get_start_point()
        self.start_position = None
        self.end_x, self.end_y = self.map.get_end_point()
        self.find_path = False
        self.path = []

    def cal_GHF(self, checkpoint):
        if checkpoint.father is not None:
            G = checkpoint.father.G + 1   # 起点到父节点的花费加上父节点到本节点的花费
        else:
            G = 0
        H = abs(checkpoint.x - self.end_x) + abs(checkpoint.y - self.end_y)
        F = G + H
        return G, H, F

    def add_near_point(self, check_point):
        x, y = check_point.get_x_y()
        tmp_list = [Point(x-1, y-1), Point(x-1, y), Point(x-1, y+1),
                    Point(x, y-1), Point(x, y+1),
                    Point(x+1, y-1), Point(x+1, y), Point(x+1, y+1)]
        near_list = []
        for pi in tmp_list:
            if self.map.map.shape[0] > pi.x >= 0 and self.map.map.shape[1] > pi.y >= 0:     # 在地图范围内
                if self.map.check_grid(pi) == 100:
                    return [pi]
                elif self.map.check_grid(pi) == 1 and self.not_in_closelist(pi):
                    near_list.append(pi)

        return near_list

    def choose_min_F_point(self):
        minF = 1e10
        choosed_point = None
        for pi in self.openlist:
            if pi.F < minF:
                minF = pi.F
                choosed_point = pi
        return choosed_point

    def not_in_openlist(self, pi):
        not_in = True
        for pii in self.openlist:
            if pii.x == pi.x and pii.y == pi.y:
                not_in = False
        return not_in

    def not_in_closelist(self, pi):
        not_in = True
        for pii in self.closelist:
            if pii.x == pi.x and pii.y == pi.y:
                not_in = False
        return not_in

    def run(self):
        self.start_position = Point(self.start_x, self.start_y)
        G, H, F = self.cal_GHF(self.start_position)
        self.start_position.set_GHF(G, H, F)
        self.openlist.append(self.start_position)

        while True:
            checking_point = self.choose_min_F_point()
            if checking_point is None or self.find_path:
                print("End!")
                break
            self.openlist.remove(checking_point)
            self.closelist.append(checking_point)
            near_list = self.add_near_point(checking_point)
            for pi in near_list:
                if self.map.check_grid(pi) == 100:
                    self.find_path = True
                    # print("find path:\n{}".format(checking_point.get_x_y()))
                    self.path.append([checking_point.get_x_y()[0], checking_point.get_x_y()[1]])
                    reverse_point_father = checking_point.father
                    while reverse_point_father.father is not None:
                        # print(reverse_point_father.get_x_y())
                        self.path.append([reverse_point_father.get_x_y()[0], reverse_point_father.get_x_y()[1]])
                        reverse_point_father = reverse_point_father.father
                    break

                if self.not_in_openlist(pi):
                    pi.father = checking_point
                    G, H, F = self.cal_GHF(pi)
                    pi.set_GHF(G, H, F)
                    self.openlist.append(pi)
                else:
                    G_new = checking_point.G + 1
                    if pi.G < G_new:
                        pi.father = checking_point
                        G, H, F = self.cal_GHF(pi)
                        pi.set_GHF(G, H, F)

        # 打印路性
        print("path: ")
        print(self.path)
        self.path = self.path[::-1]
        with open('my_path.txt', 'w') as file:
            for item in self.path:
                file.write(str(item) + '\n')

    def check(self):
        for pi in self.path:
            self.map.map[pi[0], pi[1]] = 2

        fig = plt.figure("A Star Algorithm")

        cmap = plt.cm.colors.ListedColormap(['yellow', 'black', 'white', 'blue', 'green'])

        # 创建一个离散的归一化器，根据不同数值映射到不同颜色
        bounds = [-101, -99, 0, 1, 2, 99, 101]
        norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)

        # 显示二维数组
        plt.imshow(self.map.map, cmap=cmap, norm=norm)

        # 添加颜色条，以便查看数值与颜色的对应关系
        cb = plt.colorbar()

        # 显示图
        plt.show()


if __name__ == "__main__":
    astar = Astar()

    astar.run()
    astar.check()
