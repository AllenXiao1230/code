# rocket_attitude.py
from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene, QGraphicsItem
from PyQt5.QtGui import QPainter, QPen, QBrush, QPolygonF
from PyQt5.QtCore import Qt, QRectF, QPointF


class RocketItem(QGraphicsItem):
    """
    Simple rocket shape as a QGraphicsItem.
    """

    def __init__(self):
        super().__init__()
        self.setTransformOriginPoint(0, 0)

    def boundingRect(self) -> QRectF:
        return QRectF(-20, -50, 40, 100)

    def paint(self, painter: QPainter, option, widget=None):
        painter.setRenderHint(QPainter.Antialiasing)

        # Body
        painter.setPen(QPen(Qt.white, 2))
        painter.setBrush(QBrush(Qt.darkGray))
        painter.drawRect(-6, -30, 12, 60)

        # Nose cone (FIXED)
        nose = QPolygonF([
            QPointF(-6, -30),
            QPointF(0, -50),
            QPointF(6, -30),
        ])
        painter.setBrush(QBrush(Qt.lightGray))
        painter.drawPolygon(nose)

        # Fins (FIXED)
        left_fin = QPolygonF([
            QPointF(-6, 20),
            QPointF(-16, 35),
            QPointF(-6, 35),
        ])
        right_fin = QPolygonF([
            QPointF(6, 20),
            QPointF(16, 35),
            QPointF(6, 35),
        ])
        painter.setBrush(QBrush(Qt.gray))
        painter.drawPolygon(left_fin)
        painter.drawPolygon(right_fin)


class RocketAttitudeView(QGraphicsView):
    """
    Attitude display view.
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)

        self.setRenderHint(QPainter.Antialiasing)
        self.setBackgroundBrush(QBrush(Qt.black))
        self.setFrameStyle(0)

        self.rocket_item = RocketItem()
        self.scene.addItem(self.rocket_item)

        self.scene.setSceneRect(-100, -100, 200, 200)
        self.rocket_item.setPos(0, 0)

        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        self.scale(1.5, 1.5)

    def update_attitude(self, roll: float, pitch: float, yaw: float):
        # Qt rotation is clockwise, degrees
        self.rocket_item.setRotation(-yaw)
