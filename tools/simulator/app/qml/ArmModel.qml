pragma ComponentBehavior: Bound

import QtQuick
import QtQuick3D
import QtQuick3D.AssetUtils
import Simulator
import Ui.Quick3D

Node {
    id: arm

    property var nodes: []
    property var meshes: []
    property var levels: [0, 0, 0, 0, 0, 0]
    property bool ghost: false
    property int selectedJoint: -1
    property color accentColor: "#f28c28"
    property color hubColor: "#f28c28"
    property color faultColor: "#e74c3c"
    property real faultPulse: 0

    signal meshFailed(int node, string reason)

    function highlighted(joint) {
        return joint >= 0 && (joint === selectedJoint || levels[joint] === 3)
    }

    function fadeFor(meshShown, material, joint) {
        if (!meshShown)
            return 1.0
        return material === 4 && highlighted(joint) ? 0.5 : 0.0
    }

    function emissiveFor(joint) {
        if (ghost || joint < 0)
            return Qt.vector3d(0, 0, 0)
        if (levels[joint] === 3)
            return Qt.vector3d(faultColor.r * faultPulse, faultColor.g * faultPulse, faultColor.b * faultPulse)
        if (joint === selectedJoint)
            return Qt.vector3d(hubColor.r * 0.45, hubColor.g * 0.45, hubColor.b * 0.45)
        return Qt.vector3d(0, 0, 0)
    }

    Repeater3D {
        model: 7

        Node {
            id: link
            required property int index

            position: arm.nodes.length > link.index ? arm.nodes[link.index].position : Qt.vector3d(0, 0, 0)
            rotation: arm.nodes.length > link.index ? arm.nodes[link.index].rotation : Qt.quaternion(1, 0, 0, 0)

            readonly property bool meshShown: mesh.status === RuntimeLoader.Success && String(mesh.source).length > 0

            RuntimeLoader {
                id: mesh
                objectName: arm.ghost ? "" : "mesh" + link.index
                source: !arm.ghost && arm.meshes.length > link.index ? arm.meshes[link.index] : ""
                scale: Qt.vector3d(100, 100, 100)
                visible: link.meshShown
                onStatusChanged: {
                    if (status === RuntimeLoader.Error)
                        arm.meshFailed(link.index, errorString)
                }
            }

            Repeater3D {
                model: 6

                ArmPart {
                    id: part
                    required property int index

                    joint: link.index - 1
                    kind: part.index
                    geometry: ShadedMeshGeometry {
                        id: generated
                        catalog: ArmSceneState.catalog
                        key: link.index + "/" + part.index
                    }
                    visible: !generated.empty
                    scale: Qt.vector3d(100, 100, 100)
                    ghost: arm.ghost
                    fade: arm.fadeFor(link.meshShown, part.index, link.index - 1)
                    accentColor: arm.accentColor
                    hubColor: arm.hubColor
                    emissive: arm.emissiveFor(link.index - 1)
                }
            }
        }
    }
}
