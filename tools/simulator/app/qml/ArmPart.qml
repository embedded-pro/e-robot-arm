import QtQuick
import QtQuick3D

Model {
    id: part

    property int joint: -1
    property int kind: 2
    property bool ghost: false
    property real fade: 1.0
    property color accentColor: "#f28c28"
    property color hubColor: "#f28c28"
    property vector3d emissive: Qt.vector3d(0, 0, 0)

    readonly property var baseColors: ["#f2f4f7", "#8a96a5", "#3a3f46", "#fafbfc", part.hubColor, "#4a4f56"]
    readonly property var metals: [0.45, 0.5, 0.2, 0.7, 0.2, 0.0]
    readonly property var roughnesses: [0.28, 0.4, 0.5, 0.2, 0.35, 0.75]

    readonly property bool blended: ghost || fade < 1.0

    pickable: !ghost
    castsShadows: !blended
    receivesShadows: !blended

    materials: PrincipledMaterial {
        baseColor: part.ghost ? part.accentColor : part.baseColors[part.kind]
        metalness: part.ghost ? 0.0 : part.metals[part.kind]
        roughness: part.ghost ? 0.6 : part.roughnesses[part.kind]
        emissiveFactor: part.kind === 4 ? part.emissive : Qt.vector3d(0, 0, 0)
        opacity: part.ghost ? 0.3 : part.fade
        alphaMode: part.blended ? PrincipledMaterial.Blend : PrincipledMaterial.Opaque
        depthDrawMode: part.blended ? Material.NeverDepthDraw : Material.OpaqueOnlyDepthDraw
    }
}
