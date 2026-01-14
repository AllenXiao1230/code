# rocket_attitude.py
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from PyQt5.QtGui import QColor, QVector3D, QQuaternion
from PyQt5.Qt3DCore import QEntity, QTransform
from PyQt5.Qt3DExtras import Qt3DWindow, QCylinderMesh, QConeMesh, QCuboidMesh, QPhongMaterial
from PyQt5.Qt3DRender import QPointLight


class RocketAttitudeView(QWidget):
    """
    3D rocket attitude view driven by roll/pitch/yaw.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._view = Qt3DWindow()
        self._view.defaultFrameGraph().setClearColor(QColor(10, 10, 16))

        container = QWidget.createWindowContainer(self._view, self)
        container.setMinimumSize(200, 200)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(container)

        self._root = QEntity()
        self._view.setRootEntity(self._root)

        camera = self._view.camera()
        camera.lens().setPerspectiveProjection(45.0, 16.0 / 9.0, 0.1, 1000.0)
        camera.setPosition(QVector3D(0.0, 30.0, 120.0))
        camera.setViewCenter(QVector3D(0.0, 15.0, 0.0))

        light_entity = QEntity(self._root)
        light = QPointLight(light_entity)
        light.setColor(QColor(255, 255, 255))
        light.setIntensity(1.3)
        light_transform = QTransform()
        light_transform.setTranslation(QVector3D(40.0, 60.0, 80.0))
        light_entity.addComponent(light)
        light_entity.addComponent(light_transform)

        self._rocket_root = QEntity(self._root)
        self._rocket_transform = QTransform()
        self._rocket_root.addComponent(self._rocket_transform)

        self._build_rocket()
        self._add_axes()

    def _build_rocket(self):
        body = QEntity(self._rocket_root)
        body_mesh = QCylinderMesh()
        body_mesh.setRadius(4.0)
        body_mesh.setLength(60.0)
        body_mesh.setRings(16)
        body_mesh.setSlices(32)
        body_material = QPhongMaterial()
        body_material.setDiffuse(QColor(180, 180, 190))
        body_material.setSpecular(QColor(230, 230, 240))
        body_material.setShininess(80.0)
        body.addComponent(body_mesh)
        body.addComponent(body_material)

        nose = QEntity(self._rocket_root)
        nose_mesh = QConeMesh()
        nose_mesh.setTopRadius(0.0)
        nose_mesh.setBottomRadius(4.2)
        nose_mesh.setLength(18.0)
        nose_mesh.setRings(16)
        nose_mesh.setSlices(32)
        nose_material = QPhongMaterial()
        nose_material.setDiffuse(QColor(210, 210, 220))
        nose_material.setSpecular(QColor(240, 240, 250))
        nose_material.setShininess(60.0)
        nose_transform = QTransform()
        nose_transform.setTranslation(QVector3D(0.0, 39.0, 0.0))
        nose.addComponent(nose_mesh)
        nose.addComponent(nose_material)
        nose.addComponent(nose_transform)

        for angle in (0.0, 90.0, 180.0, 270.0):
            self._add_fin(angle)

    def _add_fin(self, angle_deg: float):
        pivot = QEntity(self._rocket_root)
        pivot_transform = QTransform()
        pivot_transform.setRotation(QQuaternion.fromAxisAndAngle(0.0, 1.0, 0.0, angle_deg))
        pivot.addComponent(pivot_transform)

        fin = QEntity(pivot)
        fin_mesh = QCuboidMesh()
        fin_mesh.setXExtent(2.0)
        fin_mesh.setYExtent(10.0)
        fin_mesh.setZExtent(14.0)
        fin_material = QPhongMaterial()
        fin_material.setDiffuse(QColor(130, 130, 140))
        fin_material.setSpecular(QColor(200, 200, 210))
        fin_material.setShininess(30.0)
        fin_transform = QTransform()
        fin_transform.setTranslation(QVector3D(0.0, -22.0, 9.0))
        fin.addComponent(fin_mesh)
        fin.addComponent(fin_material)
        fin.addComponent(fin_transform)

    def _add_axes(self):
        self._add_axis("x", QColor(220, 70, 70))
        self._add_axis("y", QColor(80, 200, 120))
        self._add_axis("z", QColor(80, 140, 220))

    def _add_axis(self, axis: str, color: QColor):
        length = 45.0
        radius = 0.6
        tip_len = 6.0

        axis_entity = QEntity(self._root)
        cyl = QCylinderMesh()
        cyl.setRadius(radius)
        cyl.setLength(length)
        cyl.setRings(12)
        cyl.setSlices(20)
        mat = QPhongMaterial()
        mat.setDiffuse(color)
        mat.setSpecular(QColor(230, 230, 230))
        mat.setShininess(40.0)

        rot = QQuaternion()
        pos = QVector3D()
        if axis == "x":
            rot = QQuaternion.fromAxisAndAngle(0.0, 0.0, 1.0, -90.0)
            pos = QVector3D(length * 0.5, 0.0, 0.0)
        elif axis == "y":
            rot = QQuaternion()
            pos = QVector3D(0.0, length * 0.5, 0.0)
        else:
            rot = QQuaternion.fromAxisAndAngle(1.0, 0.0, 0.0, 90.0)
            pos = QVector3D(0.0, 0.0, length * 0.5)

        cyl_transform = QTransform()
        cyl_transform.setRotation(rot)
        cyl_transform.setTranslation(pos)
        axis_entity.addComponent(cyl)
        axis_entity.addComponent(mat)
        axis_entity.addComponent(cyl_transform)

        tip_entity = QEntity(self._root)
        tip = QConeMesh()
        tip.setTopRadius(0.0)
        tip.setBottomRadius(radius * 2.2)
        tip.setLength(tip_len)
        tip.setRings(12)
        tip.setSlices(20)
        tip_transform = QTransform()
        tip_transform.setRotation(rot)
        if axis == "x":
            tip_transform.setTranslation(QVector3D(length + tip_len * 0.5, 0.0, 0.0))
        elif axis == "y":
            tip_transform.setTranslation(QVector3D(0.0, length + tip_len * 0.5, 0.0))
        else:
            tip_transform.setTranslation(QVector3D(0.0, 0.0, length + tip_len * 0.5))

        tip_entity.addComponent(tip)
        tip_entity.addComponent(mat)
        tip_entity.addComponent(tip_transform)

    def update_attitude(self, roll: float, pitch: float, yaw: float):
        # Axis mapping: roll->Z, pitch->X, yaw->Y (rocket frame).
        q_roll = QQuaternion.fromAxisAndAngle(0.0, 0.0, 1.0, roll)
        q_pitch = QQuaternion.fromAxisAndAngle(1.0, 0.0, 0.0, pitch)
        q_yaw = QQuaternion.fromAxisAndAngle(0.0, 1.0, 0.0, yaw)
        self._rocket_transform.setRotation(q_yaw * q_pitch * q_roll)
