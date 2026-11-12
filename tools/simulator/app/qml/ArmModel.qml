import QtQuick
import QtQuick3D

Node {
    id: arm

    property var angles: [0, 0, 0, 0, 0, 0]
    property var levels: [0, 0, 0, 0, 0, 0]
    property bool ghost: false
    property int selectedJoint: -1
    property color accentColor: "#f28c28"
    property color faultColor: "#e74c3c"
    property real faultPulse: 0

    function emissiveFor(joint) {
        if (ghost)
            return Qt.vector3d(0, 0, 0)
        if (levels[joint] === 3)
            return Qt.vector3d(faultColor.r * faultPulse, faultColor.g * faultPulse, faultColor.b * faultPulse)
        if (joint === selectedJoint)
            return Qt.vector3d(accentColor.r * 0.35, accentColor.g * 0.35, accentColor.b * 0.35)
        return Qt.vector3d(0, 0, 0)
    }

    PrincipledMaterial {
        id: paint
        baseColor: arm.ghost ? arm.accentColor : "#eceff1"
        metalness: 0.0
        roughness: arm.ghost ? 0.6 : 0.42
        opacity: arm.ghost ? 0.3 : 1.0
        alphaMode: arm.ghost ? PrincipledMaterial.Blend : PrincipledMaterial.Opaque
        depthDrawMode: arm.ghost ? Material.NeverDepthDraw : Material.OpaqueOnlyDepthDraw
    }

    PrincipledMaterial {
        id: accentPaint
        baseColor: arm.accentColor
        metalness: 0.0
        roughness: 0.4
        opacity: arm.ghost ? 0.3 : 1.0
        alphaMode: arm.ghost ? PrincipledMaterial.Blend : PrincipledMaterial.Opaque
        depthDrawMode: arm.ghost ? Material.NeverDepthDraw : Material.OpaqueOnlyDepthDraw
    }

    PrincipledMaterial {
        id: cast
        baseColor: arm.ghost ? arm.accentColor : "#3c4043"
        metalness: arm.ghost ? 0.0 : 0.6
        roughness: 0.55
        opacity: arm.ghost ? 0.3 : 1.0
        alphaMode: arm.ghost ? PrincipledMaterial.Blend : PrincipledMaterial.Opaque
        depthDrawMode: arm.ghost ? Material.NeverDepthDraw : Material.OpaqueOnlyDepthDraw
    }

    PrincipledMaterial {
        id: rubber
        baseColor: arm.ghost ? arm.accentColor : "#1d1f21"
        metalness: 0.0
        roughness: 0.8
        opacity: arm.ghost ? 0.3 : 1.0
        alphaMode: arm.ghost ? PrincipledMaterial.Blend : PrincipledMaterial.Opaque
        depthDrawMode: arm.ghost ? Material.NeverDepthDraw : Material.OpaqueOnlyDepthDraw
    }

    ArmPart {
        joint: 0
        ghost: arm.ghost
        source: "#Cylinder"
        position: Qt.vector3d(0, 6, 0)
        scale: Qt.vector3d(0.24, 0.12, 0.24)
        materials: [cast]
    }

    Node {
        position: Qt.vector3d(0, 12, 0)
        eulerRotation.y: arm.angles[0]

        ArmHousing {
            joint: 0
            ghost: arm.ghost
            ghostColor: arm.accentColor
            emissive: arm.emissiveFor(0)
            position: Qt.vector3d(0, 2, 0)
            scale: Qt.vector3d(0.18, 0.04, 0.18)
        }

        ArmPart {
            joint: 0
            ghost: arm.ghost
            source: "#Cube"
            position: Qt.vector3d(0, 16, 0)
            scale: Qt.vector3d(0.14, 0.28, 0.14)
            materials: [accentPaint]
        }

        Node {
            position: Qt.vector3d(0, 32, 0)
            eulerRotation.z: arm.angles[1]

            ArmHousing {
                joint: 1
                ghost: arm.ghost
                ghostColor: arm.accentColor
                emissive: arm.emissiveFor(1)
                eulerRotation.x: 90
                scale: Qt.vector3d(0.16, 0.18, 0.16)
            }

            ArmPart {
                joint: 1
                ghost: arm.ghost
                source: "#Cube"
                position: Qt.vector3d(0, 22.5, 0)
                scale: Qt.vector3d(0.10, 0.45, 0.10)
                materials: [paint]
            }

            Node {
                position: Qt.vector3d(0, 45, 0)
                eulerRotation.z: arm.angles[2]

                ArmHousing {
                    joint: 2
                    ghost: arm.ghost
                    ghostColor: arm.accentColor
                    emissive: arm.emissiveFor(2)
                    eulerRotation.x: 90
                    scale: Qt.vector3d(0.14, 0.16, 0.14)
                }

                ArmPart {
                    joint: 2
                    ghost: arm.ghost
                    source: "#Cube"
                    position: Qt.vector3d(15, 0, 0)
                    scale: Qt.vector3d(0.30, 0.09, 0.09)
                    materials: [paint]
                }

                Node {
                    position: Qt.vector3d(30, 0, 0)
                    eulerRotation.x: arm.angles[3]

                    ArmHousing {
                        joint: 3
                        ghost: arm.ghost
                        ghostColor: arm.accentColor
                        emissive: arm.emissiveFor(3)
                        eulerRotation.z: 90
                        scale: Qt.vector3d(0.11, 0.03, 0.11)
                    }

                    ArmPart {
                        joint: 3
                        ghost: arm.ghost
                        source: "#Cube"
                        position: Qt.vector3d(5, 0, 0)
                        scale: Qt.vector3d(0.10, 0.08, 0.08)
                        materials: [paint]
                    }

                    Node {
                        position: Qt.vector3d(10, 0, 0)
                        eulerRotation.z: arm.angles[4]

                        ArmHousing {
                            joint: 4
                            ghost: arm.ghost
                            ghostColor: arm.accentColor
                            emissive: arm.emissiveFor(4)
                            eulerRotation.x: 90
                            scale: Qt.vector3d(0.12, 0.12, 0.12)
                        }

                        ArmPart {
                            joint: 4
                            ghost: arm.ghost
                            source: "#Cylinder"
                            position: Qt.vector3d(5, 0, 0)
                            eulerRotation.z: 90
                            scale: Qt.vector3d(0.08, 0.10, 0.08)
                            materials: [paint]
                        }

                        Node {
                            position: Qt.vector3d(10, 0, 0)
                            eulerRotation.x: arm.angles[5]

                            ArmHousing {
                                joint: 5
                                ghost: arm.ghost
                                ghostColor: arm.accentColor
                                emissive: arm.emissiveFor(5)
                                position: Qt.vector3d(1, 0, 0)
                                eulerRotation.z: 90
                                scale: Qt.vector3d(0.07, 0.02, 0.07)
                            }

                            ArmPart {
                                joint: 5
                                ghost: arm.ghost
                                source: "#Cylinder"
                                position: Qt.vector3d(3.5, 0, 0)
                                eulerRotation.z: 90
                                scale: Qt.vector3d(0.06, 0.03, 0.06)
                                materials: [rubber]
                            }

                            ArmPart {
                                joint: 5
                                ghost: arm.ghost
                                source: "#Cube"
                                position: Qt.vector3d(5.5, 2.4, 0)
                                scale: Qt.vector3d(0.012, 0.012, 0.012)
                                materials: [accentPaint]
                            }
                        }
                    }
                }
            }
        }
    }
}
