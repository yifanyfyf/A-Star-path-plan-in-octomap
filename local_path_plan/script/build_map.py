from PyQt5.QtWidgets import QApplication, QWidget, QPushButton
from PyQt5.QtGui import QPainter, QPen, QColor, QBrush
from PyQt5.QtCore import Qt
import numpy as np
import sys
import json


class config:
    WIDTH = 20  # 地图列数
    HEIGHT = 20  # 地图行数
    blockLength = 30  # 绘制画面时每一个节点方块的边长


class CustomWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.Map = []
        for i in range(config.HEIGHT):
            col = []
            for j in range(config.WIDTH):
                col.append(1)
            self.Map.append(col)

        self.text = "Hello Build-Map"
        self.button_saveMap = QPushButton("Save Map", self)
        self.button_saveMap.resize(80, 30)
        self.button_saveMap.move(100 + (config.WIDTH - 1) * config.blockLength, 350)
        self.button_saveMap.clicked.connect(self.button_SaveMap)

        self.button_loadMap = QPushButton("Load Map", self)
        self.button_loadMap.resize(80, 30)
        self.button_loadMap.move(200 + (config.WIDTH - 1) * config.blockLength, 350)
        self.button_loadMap.clicked.connect(self.button_LoadMap)

        self.button_clearWall = QPushButton("Clean", self)
        self.button_clearWall.move(100 + (config.WIDTH - 1) * config.blockLength, 300)
        self.button_clearWall.clicked.connect(self.button_Clear)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setPen(QPen(QColor(0, 0, 0)))
        painter.drawText(15, 15, self.text)
        for i in range(config.HEIGHT):
            for j in range(config.WIDTH):
                if self.Map[i][j] == 0:
                    painter.setBrush(QBrush(QColor(0, 0, 0)))  # 使用黑色的 QBrush 填充
                else:
                    painter.setPen(QPen(QColor(0, 0, 0)))
                    painter.setBrush(QBrush(QColor(255, 255, 255)))  # 取消掉黑色的 QBrush 填充
                painter.drawRect(50 + j * config.blockLength, 50 + i * config.blockLength, config.blockLength,
                             config.blockLength)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            x, y = event.x() - 50, event.y() - 50
            x = x // config.blockLength
            y = y // config.blockLength
            if 0 <= x < config.WIDTH and 0 <= y < config.HEIGHT:
                self.Map[y][x] = (0 if self.Map[y][x] == 1 else 1)

            self.update()  # 通知窗口重新绘制

    def button_SaveMap(self):
        map_np = np.array(self.Map)
        # 指定要保存的文件名
        file_name = 'my_map222.txt'
        # 使用 savetxt 函数保存数组
        np.savetxt(file_name, map_np, fmt='%d')
        self.text = 'Map saved'
        self.update()

    def button_LoadMap(self):
        try:
            # 从文本文件读取数据到NumPy数组
            data = np.loadtxt('my_map222.txt')
            # 将NumPy数组转换为Python二维列表
            self.Map = data.tolist()
            self.text = 'Map loaded'

        except Exception as e:
            print('Fail', e, type(e))
            if type(e) == FileNotFoundError:
                self.text = 'Fail: Not found'
            elif type(e) == json.decoder.JSONDecodeError:
                self.text = 'Fail: format error'

        self.update()

    def button_Clear(self):
        for i in range(config.HEIGHT):
            for j in range(config.WIDTH):
                self.Map[i][j] = 1
        self.update()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = CustomWidget()
    window.setMinimumSize(150 + (config.WIDTH * config.blockLength - config.blockLength) + 200,
                            150 + (config.HEIGHT * config.blockLength - config.blockLength))
    window.setMaximumSize(150 + (config.WIDTH * config.blockLength - config.blockLength) + 200,
                            150 + (config.HEIGHT * config.blockLength - config.blockLength))

    window.show()
    sys.exit(app.exec_())
