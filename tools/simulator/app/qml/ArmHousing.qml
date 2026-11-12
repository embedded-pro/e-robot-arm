import QtQuick
import QtQuick3D

ArmPart {
    id: housing

    property color ghostColor: "#f28c28"
    property vector3d emissive: Qt.vector3d(0, 0, 0)

    source: "#Cylinder"
    materials: PrincipledMaterial {
        baseColor: housing.ghost ? housing.ghostColor : "#9aa0a6"
        metalness: housing.ghost ? 0.0 : 1.0
        roughness: 0.35
        emissiveFactor: housing.emissive
        opacity: housing.ghost ? 0.3 : 1.0
        alphaMode: housing.ghost ? PrincipledMaterial.Blend : PrincipledMaterial.Opaque
        depthDrawMode: housing.ghost ? Material.NeverDepthDraw : Material.OpaqueOnlyDepthDraw
    }
}
