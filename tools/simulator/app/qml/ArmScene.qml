pragma ComponentBehavior: Bound

import QtQml
import QtQuick
import QtQuick3D
import Simulator
import Ui.Quick3D

Item {
    id: root

    property real pulse

    SequentialAnimation on pulse {
        running: ArmSceneState.anyFault
        loops: Animation.Infinite
        NumberAnimation { from: 0.1; to: 0.9; duration: 600; easing.type: Easing.InOutSine }
        NumberAnimation { from: 0.9; to: 0.1; duration: 600; easing.type: Easing.InOutSine }
    }

    StudioView {
        id: studio
        objectName: "studio"
        anchors.fill: parent
        focus: true

        extent: ArmSceneState.extent
        backgroundColor: ArmSceneState.backgroundColor
        floorColor: ArmSceneState.floorColor
        gridColor: ArmSceneState.gridColor

        onPicked: (result) => ArmSceneState.Pick(result.objectHit && result.objectHit.joint !== undefined ? result.objectHit.joint : -1)

        ArmModel {
            id: measuredArm
            nodes: ArmSceneState.measuredNodes
            meshes: ArmSceneState.meshes
            onMeshFailed: (node, reason) => ArmSceneState.ReportMeshFailure(node, reason)
            levels: ArmSceneState.levels
            selectedJoint: ArmSceneState.selectedJoint
            accentColor: ArmSceneState.accentColor
            faultColor: ArmSceneState.faultColor
            faultPulse: root.pulse
        }

        Node {
            objectName: "toolTriad"
            visible: ArmSceneState.trail.length > 0
            position: ArmSceneState.toolPosition
            rotation: ArmSceneState.toolRotation

            AxisTriad {
                length: ArmSceneState.extent * 0.12
            }
        }

        Model {
            objectName: "toolTrail"
            visible: ArmSceneState.trail.length > 1
            castsShadows: false
            receivesShadows: false
            pickable: false
            geometry: PolylineGeometry {
                points: ArmSceneState.trail
            }
            materials: DefaultMaterial {
                lighting: DefaultMaterial.NoLighting
                diffuseColor: ArmSceneState.accentColor
            }
        }

        Repeater3D {
            model: ArmSceneState.boxes

            Node {
                id: box
                required property var modelData

                objectName: box.modelData.zone ? "forbiddenZone" : "workspaceBox"
                readonly property color tone: box.modelData.violated ? ArmSceneState.faultColor
                                                                     : (box.modelData.zone ? ArmSceneState.warningColor : ArmSceneState.gridColor)

                Model {
                    visible: box.modelData.zone
                    source: "#Cube"
                    position: box.modelData.centre
                    scale: Qt.vector3d(box.modelData.size.x / 100, box.modelData.size.y / 100, box.modelData.size.z / 100)
                    castsShadows: false
                    receivesShadows: false
                    pickable: false
                    materials: PrincipledMaterial {
                        lighting: PrincipledMaterial.NoLighting
                        baseColor: box.tone
                        opacity: box.modelData.violated ? 0.35 : 0.18
                        alphaMode: PrincipledMaterial.Blend
                        depthDrawMode: Material.NeverDepthDraw
                    }
                }

                Model {
                    castsShadows: false
                    receivesShadows: false
                    pickable: false
                    geometry: PolylineGeometry {
                        points: box.modelData.outline
                    }
                    materials: DefaultMaterial {
                        lighting: DefaultMaterial.NoLighting
                        diffuseColor: box.tone
                    }
                }
            }
        }

        ArmModel {
            id: ghostArm
            visible: ArmSceneState.ghostVisible
            ghost: true
            nodes: ArmSceneState.commandedNodes
            accentColor: ArmSceneState.accentColor
        }
    }

    StatusBadge {
        objectName: "singularityWarning"
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.margins: 12
        text: ArmSceneState.singularityWarning
        tone: ArmSceneState.warningColor
    }

    Connections {
        target: ArmSceneState
        function onCameraPresetRequested(preset) {
            studio.showPreset(preset)
        }
    }
}
