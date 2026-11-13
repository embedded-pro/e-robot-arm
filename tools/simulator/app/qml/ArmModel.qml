pragma ComponentBehavior: Bound

import QtQuick
import QtQuick3D

Node {
    id: arm

    property var nodes: []
    property var parts: []
    property var levels: [0, 0, 0, 0, 0, 0]
    property bool ghost: false
    property int selectedJoint: -1
    property color accentColor: "#f28c28"
    property color faultColor: "#e74c3c"
    property real faultPulse: 0

    function emissiveFor(joint) {
        if (ghost || joint < 0)
            return Qt.vector3d(0, 0, 0)
        if (levels[joint] === 3)
            return Qt.vector3d(faultColor.r * faultPulse, faultColor.g * faultPulse, faultColor.b * faultPulse)
        if (joint === selectedJoint)
            return Qt.vector3d(accentColor.r * 0.35, accentColor.g * 0.35, accentColor.b * 0.35)
        return Qt.vector3d(0, 0, 0)
    }

    Repeater3D {
        model: 7

        Node {
            id: link
            required property int index

            position: arm.nodes.length > link.index ? arm.nodes[link.index].position : Qt.vector3d(0, 0, 0)
            rotation: arm.nodes.length > link.index ? arm.nodes[link.index].rotation : Qt.quaternion(1, 0, 0, 0)

            Repeater3D {
                model: arm.parts.length > link.index ? arm.parts[link.index] : []

                ArmPart {
                    required property var modelData

                    joint: modelData.joint
                    kind: modelData.material
                    source: modelData.source
                    position: modelData.position
                    rotation: modelData.rotation
                    scale: modelData.scale
                    ghost: arm.ghost
                    accentColor: arm.accentColor
                    emissive: arm.emissiveFor(modelData.joint)
                }
            }
        }
    }
}
