import QtQml
import QtQuick
import QtQuick3D
import QtQuick3D.Helpers
import Simulator

Rectangle {
    id: root

    color: ArmSceneState.backgroundColor
    focus: true

    property real pulse

    SequentialAnimation on pulse {
        running: ArmSceneState.anyFault
        loops: Animation.Infinite
        NumberAnimation { from: 0.1; to: 0.9; duration: 600; easing.type: Easing.InOutSine }
        NumberAnimation { from: 0.9; to: 0.1; duration: 600; easing.type: Easing.InOutSine }
    }

    View3D {
        id: view
        anchors.fill: parent
        camera: camera

        environment: SceneEnvironment {
            objectName: "environment"
            backgroundMode: SceneEnvironment.Color
            clearColor: ArmSceneState.backgroundColor
            lightProbe: Texture { source: "qrc:/Simulator/assets/studio.hdr" }
            probeExposure: 0.8
            tonemapMode: SceneEnvironment.TonemapModeFilmic
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.High
            specularAAEnabled: true
            aoStrength: 55
            aoDistance: 5
            aoSoftness: 40
            aoDither: true
            depthPrePassEnabled: true
        }

        Node {
            id: pivot
            position: Qt.vector3d(0, ArmSceneState.extent * 0.45, 0)
            eulerRotation: Qt.vector3d(-18, 35, 0)

            PerspectiveCamera {
                id: camera
                position: Qt.vector3d(0, 0, ArmSceneState.extent * 2.4)
                fieldOfView: 40
                clipNear: 5
                clipFar: 2000
            }
        }

        DirectionalLight {
            id: keyLight
            eulerRotation: Qt.vector3d(-50, -35, 0)
            brightness: 1.0
            castsShadow: true
            shadowMapQuality: Light.ShadowMapQualityVeryHigh
            shadowMapFar: 600
            shadowFactor: 70
            shadowFilter: 10
        }

        DirectionalLight {
            eulerRotation: Qt.vector3d(-15, 140, 0)
            brightness: 0.3
        }

        DirectionalLight {
            eulerRotation: Qt.vector3d(-60, 180, 0)
            brightness: 0.5
        }

        Model {
            source: "#Rectangle"
            eulerRotation.x: -90
            scale: Qt.vector3d(3, 3, 1)
            receivesShadows: true
            castsShadows: false
            pickable: false
            materials: PrincipledMaterial {
                baseColor: ArmSceneState.floorColor
                metalness: 0.0
                roughness: 0.9
            }
        }

        Repeater3D {
            model: 31

            Node {
                id: gridLine
                required property int index

                Model {
                    source: "#Cube"
                    position: Qt.vector3d(-150 + gridLine.index * 10, 0.05, 0)
                    scale: Qt.vector3d(0.0015, 0.0005, 3)
                    castsShadows: false
                    pickable: false
                    materials: DefaultMaterial {
                        lighting: DefaultMaterial.NoLighting
                        diffuseColor: ArmSceneState.gridColor
                        opacity: 0.35
                    }
                }

                Model {
                    source: "#Cube"
                    position: Qt.vector3d(0, 0.05, -150 + gridLine.index * 10)
                    scale: Qt.vector3d(3, 0.0005, 0.0015)
                    castsShadows: false
                    pickable: false
                    materials: DefaultMaterial {
                        lighting: DefaultMaterial.NoLighting
                        diffuseColor: ArmSceneState.gridColor
                        opacity: 0.35
                    }
                }
            }
        }

        ArmModel {
            id: measuredArm
            nodes: ArmSceneState.measuredNodes
            parts: ArmSceneState.parts
            levels: ArmSceneState.levels
            selectedJoint: ArmSceneState.selectedJoint
            accentColor: ArmSceneState.accentColor
            faultColor: ArmSceneState.faultColor
            faultPulse: root.pulse
        }

        ArmModel {
            id: ghostArm
            visible: ArmSceneState.ghostVisible
            ghost: true
            nodes: ArmSceneState.commandedNodes
            parts: ArmSceneState.parts
            accentColor: ArmSceneState.accentColor
        }
    }

    OrbitCameraController {
        anchors.fill: parent
        origin: pivot
        camera: camera
    }

    TapHandler {
        onTapped: (eventPoint) => {
            root.forceActiveFocus()
            const result = view.pick(eventPoint.position.x, eventPoint.position.y)
            ArmSceneState.Pick(result.objectHit && result.objectHit.joint !== undefined ? result.objectHit.joint : -1)
        }
        onDoubleTapped: root.frame()
    }

    function goTo(rotation, distance) {
        rotationAnimation.to = rotation
        distanceAnimation.to = Qt.vector3d(0, 0, distance * ArmSceneState.extent)
        cameraMove.restart()
    }

    function frame() {
        goTo(Qt.vector3d(-18, 35, 0), 2.4)
    }

    ParallelAnimation {
        id: cameraMove
        PropertyAnimation { id: rotationAnimation; target: pivot; property: "eulerRotation"; duration: 250; easing.type: Easing.OutCubic }
        PropertyAnimation { id: distanceAnimation; target: camera; property: "position"; duration: 250; easing.type: Easing.OutCubic }
    }

    Connections {
        target: ArmSceneState
        function onCameraPresetRequested(preset) {
            switch (preset) {
            case 0: root.goTo(Qt.vector3d(-10, 0, 0), 2.9); break
            case 1: root.goTo(Qt.vector3d(-10, 90, 0), 2.9); break
            case 2: root.goTo(Qt.vector3d(-89, 0, 0), 3.3); break
            case 3: root.goTo(Qt.vector3d(-30, 45, 0), 3.1); break
            default: root.frame(); break
            }
        }
    }

    Keys.onPressed: (event) => {
        if (event.key >= Qt.Key_1 && event.key <= Qt.Key_4) {
            ArmSceneState.RequestCameraPreset(event.key - Qt.Key_1)
            event.accepted = true
        } else if (event.key === Qt.Key_F) {
            root.frame()
            event.accepted = true
        }
    }
}
