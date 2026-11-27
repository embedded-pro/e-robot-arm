pragma ComponentBehavior: Bound

import QtQml
import QtQuick
import QtQuick3D
import Simulator
import Ui.Quick3D

Item {
    id: root

    property real pulse
    property bool framePending: true

    SequentialAnimation on pulse {
        running: ArmSceneState.anyFault
        loops: Animation.Infinite
        NumberAnimation { from: 0.1; to: 0.9; duration: 600; easing.type: Easing.InOutSine }
        NumberAnimation { from: 0.9; to: 0.1; duration: 600; easing.type: Easing.InOutSine }
    }

    component Entry: Row {
        id: entry
        property string label
        property color tone
        property bool line: true
        property real swatchOpacity: 1.0
        spacing: 6
        Item {
            width: 18
            height: 12
            Rectangle {
                anchors.verticalCenter: parent.verticalCenter
                width: parent.width
                height: entry.line ? 2 : parent.height
                radius: entry.line ? 0 : 2
                color: entry.tone
                opacity: entry.swatchOpacity
                border.color: entry.line ? "transparent" : Qt.darker(entry.tone, 1.6)
                border.width: entry.line ? 0 : 1
            }
        }
        Text {
            text: entry.label
            color: ArmSceneState.textColor
            font.pixelSize: 11
        }
    }

    function frameWhenSized() {
        if (studio.width > 0 && studio.height > 0 && ArmSceneState.extent > 0) {
            root.framePending = false
            studio.frame()
        }
    }

    StudioView {
        id: studio
        objectName: "studio"
        anchors.fill: parent
        focus: true

        extent: ArmSceneState.extent
        bounds: ArmSceneState.frameBounds
        orthographic: ArmSceneState.orthographic
        backgroundColor: ArmSceneState.backgroundColor
        floorColor: ArmSceneState.floorColor
        gridColor: ArmSceneState.gridColor
        gizmoTextColor: "#ffffff"
        gizmoAxes: [
            { label: "X", direction: Qt.vector3d(1, 0, 0), color: "#e53935" },
            { label: "Y", direction: Qt.vector3d(0, 0, -1), color: "#43a047" },
            { label: "Z", direction: Qt.vector3d(0, 1, 0), color: "#1e88e5" }
        ]

        onWidthChanged: if (root.framePending) root.frameWhenSized()
        onHeightChanged: if (root.framePending) root.frameWhenSized()

        onPicked: (result) => ArmSceneState.Pick(result.objectHit && result.objectHit.joint !== undefined ? result.objectHit.joint : -1)

        Keys.onReleased: (event) => {
            if (event.key === Qt.Key_Control && guideArea.guiding) {
                guideArea.guiding = false
                ArmSceneState.GuideRelease()
            } else if (event.key === Qt.Key_Shift && ArmSceneState.dragFollowing) {
                guideArea.dragging = false
                ArmSceneState.DragRelease()
            } else if (event.key === Qt.Key_Shift && ArmSceneState.gizmoFollowing) {
                guideArea.turning = false
                ArmSceneState.GizmoRelease()
            }
        }

        Keys.onPressed: (event) => {
            if (event.key === Qt.Key_O) {
                ArmSceneState.orthographic = !ArmSceneState.orthographic
                event.accepted = true
            } else if (event.key === Qt.Key_J) {
                ArmSceneState.RequestFocusSelected()
                event.accepted = true
            } else if (event.key === Qt.Key_G) {
                ArmSceneState.gizmoShown = !ArmSceneState.gizmoShown
                event.accepted = true
            } else if ((event.key === Qt.Key_Return || event.key === Qt.Key_Enter) && (ArmSceneState.dragActive || ArmSceneState.gizmoActive)) {
                ArmSceneState.DragConfirm()
                event.accepted = true
            } else if (event.key === Qt.Key_Delete && (ArmSceneState.dragActive || ArmSceneState.gizmoActive)) {
                ArmSceneState.DragClear()
                event.accepted = true
            }
        }

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
            visible: ArmSceneState.showFrames && ArmSceneState.trail.length > 0
            position: ArmSceneState.toolPosition
            rotation: ArmSceneState.toolRotation

            AxisTriad {
                length: ArmSceneState.extent * 0.12
            }
        }

        Model {
            objectName: "guideArrow"
            visible: ArmSceneState.guideActive && ArmSceneState.guideArrow.length > 1
            castsShadows: false
            receivesShadows: false
            pickable: false
            geometry: PolylineGeometry {
                points: ArmSceneState.guideArrow
            }
            materials: DefaultMaterial {
                lighting: DefaultMaterial.NoLighting
                diffuseColor: ArmSceneState.accentColor
            }
        }

        Model {
            objectName: "guidePoint"
            visible: ArmSceneState.guideActive && ArmSceneState.guideArrow.length > 1
            source: "#Sphere"
            position: ArmSceneState.guidePoint
            scale: Qt.vector3d(0.025, 0.025, 0.025)
            castsShadows: false
            receivesShadows: false
            pickable: false
            materials: DefaultMaterial {
                lighting: DefaultMaterial.NoLighting
                diffuseColor: ArmSceneState.accentColor
            }
        }

        Model {
            objectName: "toolTrail"
            visible: ArmSceneState.showTrail && ArmSceneState.trail.length > 1
            castsShadows: false
            receivesShadows: false
            pickable: false
            geometry: PolylineGeometry {
                points: ArmSceneState.trail
            }
            materials: DefaultMaterial {
                lighting: DefaultMaterial.NoLighting
                diffuseColor: ArmSceneState.trailColor
            }
        }

        Node {
            objectName: "targetMarker"
            visible: ArmSceneState.showFrames && ArmSceneState.targetVisible
            position: ArmSceneState.targetPosition
            rotation: ArmSceneState.targetRotation

            AxisTriad {
                length: ArmSceneState.extent * 0.12 * 0.6
            }
        }

        Model {
            objectName: "limitArc"
            visible: ArmSceneState.showLimits && ArmSceneState.limitArc.length > 1
            castsShadows: false
            receivesShadows: false
            pickable: false
            geometry: PolylineGeometry {
                points: ArmSceneState.limitArc
            }
            materials: DefaultMaterial {
                lighting: DefaultMaterial.NoLighting
                diffuseColor: ArmSceneState.selectionColor
            }
        }

        Model {
            objectName: "selectionRing"
            visible: ArmSceneState.selectionRing.length > 1
            castsShadows: false
            receivesShadows: false
            pickable: false
            geometry: PolylineGeometry {
                points: ArmSceneState.selectionRing
            }
            materials: DefaultMaterial {
                lighting: DefaultMaterial.NoLighting
                diffuseColor: ArmSceneState.selectionColor
            }
        }

        Repeater3D {
            model: ArmSceneState.boxes

            Node {
                id: box
                required property var modelData

                objectName: box.modelData.zone ? "forbiddenZone" : "workspaceBox"
                visible: ArmSceneState.showWorkspace || box.modelData.violated
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
            objectName: "ghostArm"
            visible: ArmSceneState.showGhost && ArmSceneState.ghostVisible
            ghost: true
            nodes: ArmSceneState.commandedNodes
            accentColor: ArmSceneState.accentColor
        }

        ArmModel {
            id: dragGhost
            objectName: "dragGhost"
            visible: ArmSceneState.dragActive
            ghost: true
            nodes: ArmSceneState.dragNodes
            accentColor: ArmSceneState.accentColor
        }

        ArmModel {
            id: gizmoGhost
            objectName: "gizmoGhost"
            visible: ArmSceneState.gizmoActive
            ghost: true
            nodes: ArmSceneState.gizmoNodes
            accentColor: ArmSceneState.accentColor
        }

        Repeater3D {
            objectName: "toolGizmo"
            model: ArmSceneState.gizmoShown && ArmSceneState.gizmoOffered ? ArmSceneState.gizmoLines : []

            Model {
                id: handle
                required property var modelData
                required property int index
                readonly property bool active: ArmSceneState.gizmoHandle === handle.index
                readonly property var tones: ["#e53935", "#43a047", "#1e88e5"]

                castsShadows: false
                receivesShadows: false
                pickable: false
                opacity: ArmSceneState.gizmoHandle < 0 || handle.active ? 1.0 : 0.4
                geometry: PolylineGeometry {
                    points: handle.modelData
                }
                materials: DefaultMaterial {
                    lighting: DefaultMaterial.NoLighting
                    diffuseColor: handle.active && ArmSceneState.gizmoFault ? ArmSceneState.faultColor : handle.tones[handle.index % 3]
                }
            }
        }
    }

    Rectangle {
        id: jointLabel
        objectName: "jointLabel"

        readonly property vector3d screen: {
            studio.pivot.position
            studio.pivot.eulerRotation
            studio.camera.position
            studio.orthographic
            studio.width
            studio.height
            if (ArmSceneState.selectionText.length === 0 || !studio.view.camera)
                return Qt.vector3d(0, 0, -1)
            return studio.view.mapFrom3DScene(ArmSceneState.selectionAnchor)
        }
        readonly property real gap: 14

        visible: ArmSceneState.selectionText.length > 0 && jointLabel.screen.z > 0
        enabled: false
        x: Math.max(8, Math.min(root.width - width - 8, jointLabel.screen.x + jointLabel.gap))
        y: Math.max(8, Math.min(root.height - height - 8, jointLabel.screen.y - height - jointLabel.gap))
        width: labelText.implicitWidth + 16
        height: labelText.implicitHeight + 8
        radius: 3
        color: Qt.rgba(ArmSceneState.panelColor.r, ArmSceneState.panelColor.g, ArmSceneState.panelColor.b, 0.88)
        border.color: ArmSceneState.selectionColor
        border.width: 1

        Accessible.role: Accessible.StaticText
        Accessible.name: ArmSceneState.selectionText

        Text {
            id: labelText
            anchors.centerIn: parent
            text: ArmSceneState.selectionText
            color: ArmSceneState.textColor
            font.family: "monospace"
            font.pixelSize: 12
        }
    }

    MouseArea {
        id: guideArea
        objectName: "guideArea"

        property bool guiding: false
        property bool turning: false
        property bool dragging: false
        property bool moved: false
        property int pressJoint: -1
        property point pressAt: Qt.point(0, 0)

        function ray(x, y) {
            const near = studio.view.mapTo3DScene(Qt.vector3d(x, y, 0))
            const far = studio.view.mapTo3DScene(Qt.vector3d(x, y, 100))
            return [near, far.minus(near)]
        }

        function linkAt(x, y) {
            const hit = studio.view.pick(x, y)
            return hit.objectHit && hit.objectHit.joint !== undefined && hit.objectHit.joint >= 0 ? hit : null
        }

        anchors.fill: parent
        enabled: ArmSceneState.guideOffered || ArmSceneState.dragOffered || ArmSceneState.gizmoShown
        acceptedButtons: Qt.LeftButton | Qt.RightButton
        cursorShape: guideArea.guiding || guideArea.dragging || guideArea.turning ? Qt.ClosedHandCursor : Qt.ArrowCursor

        onPressed: (mouse) => {
            mouse.accepted = false
            if (mouse.button === Qt.RightButton) {
                if (ArmSceneState.dragActive || ArmSceneState.gizmoActive) {
                    ArmSceneState.DragClear()
                    mouse.accepted = true
                }
                return
            }
            if (!(mouse.modifiers & Qt.ControlModifier) && guideArea.grabGizmo(mouse)) {
                mouse.accepted = true
                return
            }
            const hit = guideArea.linkAt(mouse.x, mouse.y)
            if (!hit)
                return
            const guide = (mouse.modifiers & Qt.ControlModifier) !== 0
            if (guide ? !ArmSceneState.guideOffered : !ArmSceneState.dragOffered)
                return
            const pointer = guideArea.ray(mouse.x, mouse.y)
            studio.forceActiveFocus()
            mouse.accepted = true
            if (guide) {
                guideArea.guiding = true
                ArmSceneState.GuideGrab(hit.objectHit.joint, hit.scenePosition, pointer[0], pointer[1])
                return
            }
            guideArea.dragging = true
            guideArea.moved = false
            guideArea.pressJoint = hit.objectHit.joint
            guideArea.pressAt = Qt.point(mouse.x, mouse.y)
            ArmSceneState.DragGrab(hit.objectHit.joint, hit.scenePosition, pointer[0], pointer[1], (mouse.modifiers & Qt.ShiftModifier) !== 0)
        }
        function grabGizmo(mouse) {
            const pointer = guideArea.ray(mouse.x, mouse.y)
            const grabbed = ArmSceneState.GizmoHandleAt(pointer[0], pointer[1])
            if (grabbed < 0)
                return false
            studio.forceActiveFocus()
            guideArea.turning = true
            ArmSceneState.GizmoGrab(grabbed, pointer[0], pointer[1], (mouse.modifiers & Qt.ShiftModifier) !== 0)
            return true
        }

        onPositionChanged: (mouse) => {
            if (guideArea.dragging && !guideArea.moved)
                guideArea.moved = Math.abs(mouse.x - guideArea.pressAt.x) + Math.abs(mouse.y - guideArea.pressAt.y) > 4
            if (!guideArea.guiding && !guideArea.turning && !(guideArea.dragging && guideArea.moved))
                return
            const pointer = guideArea.ray(mouse.x, mouse.y)
            ArmSceneState.GuidePointer(pointer[0], pointer[1])
        }
        onReleased: guideArea.stop()
        onCanceled: guideArea.stop()

        function stop() {
            if (guideArea.guiding) {
                guideArea.guiding = false
                ArmSceneState.GuideRelease()
            }
            if (guideArea.turning) {
                guideArea.turning = false
                ArmSceneState.GizmoRelease()
            }
            if (!guideArea.dragging)
                return
            guideArea.dragging = false
            if (guideArea.moved) {
                ArmSceneState.DragRelease()
                return
            }
            ArmSceneState.DragClear()
            ArmSceneState.Pick(guideArea.pressJoint)
        }
    }

    Rectangle {
        id: dragLabel
        objectName: "dragLabel"

        readonly property vector3d screen: {
            studio.pivot.position
            studio.pivot.eulerRotation
            studio.camera.position
            studio.orthographic
            studio.width
            studio.height
            if (!ArmSceneState.dragActive || !studio.view.camera)
                return Qt.vector3d(0, 0, -1)
            return studio.view.mapFrom3DScene(ArmSceneState.dragPoint)
        }

        visible: ArmSceneState.dragActive && ArmSceneState.dragLabel.length > 0 && dragLabel.screen.z > 0
        enabled: false
        x: Math.max(8, Math.min(root.width - width - 8, dragLabel.screen.x + 14))
        y: Math.max(8, Math.min(root.height - height - 8, dragLabel.screen.y + 14))
        width: dragText.implicitWidth + 16
        height: dragText.implicitHeight + 8
        radius: 3
        color: Qt.rgba(ArmSceneState.panelColor.r, ArmSceneState.panelColor.g, ArmSceneState.panelColor.b, 0.88)
        border.color: ArmSceneState.accentColor
        border.width: 1

        Accessible.role: Accessible.StaticText
        Accessible.name: ArmSceneState.dragLabel

        Text {
            id: dragText
            anchors.centerIn: parent
            text: ArmSceneState.dragLabel
            color: ArmSceneState.textColor
            font.family: "monospace"
            font.pixelSize: 12
        }
    }

    Rectangle {
        id: gizmoLabel
        objectName: "gizmoLabel"

        readonly property vector3d screen: {
            studio.pivot.position
            studio.pivot.eulerRotation
            studio.camera.position
            studio.orthographic
            studio.width
            studio.height
            if (!ArmSceneState.gizmoActive || !studio.view.camera)
                return Qt.vector3d(0, 0, -1)
            return studio.view.mapFrom3DScene(ArmSceneState.gizmoPoint)
        }

        visible: ArmSceneState.gizmoActive && ArmSceneState.gizmoLabel.length > 0 && gizmoLabel.screen.z > 0
        enabled: false
        x: Math.max(8, Math.min(root.width - width - 8, gizmoLabel.screen.x + 14))
        y: Math.max(8, Math.min(root.height - height - 8, gizmoLabel.screen.y + 14))
        width: gizmoText.implicitWidth + 16
        height: gizmoText.implicitHeight + 8
        radius: 3
        color: Qt.rgba(ArmSceneState.panelColor.r, ArmSceneState.panelColor.g, ArmSceneState.panelColor.b, 0.88)
        border.color: ArmSceneState.gizmoLabelColor
        border.width: 1

        Accessible.role: Accessible.StaticText
        Accessible.name: ArmSceneState.gizmoLabel

        Text {
            id: gizmoText
            anchors.centerIn: parent
            text: ArmSceneState.gizmoLabel
            color: ArmSceneState.textColor
            font.family: "monospace"
            font.pixelSize: 12
        }
    }

    Rectangle {
        id: guideLabel
        objectName: "guideLabel"

        readonly property vector3d screen: {
            studio.pivot.position
            studio.pivot.eulerRotation
            studio.camera.position
            studio.orthographic
            studio.width
            studio.height
            if (!ArmSceneState.guideActive || !studio.view.camera)
                return Qt.vector3d(0, 0, -1)
            return studio.view.mapFrom3DScene(ArmSceneState.guidePoint)
        }

        visible: ArmSceneState.guideActive && ArmSceneState.guideLabel.length > 0 && guideLabel.screen.z > 0
        enabled: false
        x: Math.max(8, Math.min(root.width - width - 8, guideLabel.screen.x + 14))
        y: Math.max(8, Math.min(root.height - height - 8, guideLabel.screen.y + 14))
        width: guideText.implicitWidth + 16
        height: guideText.implicitHeight + 8
        radius: 3
        color: Qt.rgba(ArmSceneState.panelColor.r, ArmSceneState.panelColor.g, ArmSceneState.panelColor.b, 0.88)
        border.color: ArmSceneState.accentColor
        border.width: 1

        Accessible.role: Accessible.StaticText
        Accessible.name: ArmSceneState.guideLabel

        Text {
            id: guideText
            anchors.centerIn: parent
            text: ArmSceneState.guideLabel
            color: ArmSceneState.textColor
            font.family: "monospace"
            font.pixelSize: 12
        }
    }

    Rectangle {
        id: legend
        objectName: "legend"
        visible: ArmSceneState.showLegend
        enabled: false
        anchors.left: parent.left
        anchors.bottom: parent.bottom
        anchors.margins: 12
        width: legendRows.implicitWidth + 16
        height: legendRows.implicitHeight + 12
        radius: 4
        color: Qt.rgba(ArmSceneState.panelColor.r, ArmSceneState.panelColor.g, ArmSceneState.panelColor.b, 0.82)

        Accessible.role: Accessible.Grouping
        Accessible.name: qsTr("3-D view legend")

        Column {
            id: legendRows
            x: 8
            y: 6
            spacing: 3

            Entry { label: qsTr("Measured arm"); tone: "#f2f4f7"; line: false }
            Entry { visible: ArmSceneState.showGhost; label: qsTr("Commanded arm (ghost)"); tone: ArmSceneState.accentColor; line: false; swatchOpacity: 0.35 }
            Entry { visible: ArmSceneState.showTrail; label: qsTr("Tool trail"); tone: ArmSceneState.trailColor }
            Entry { visible: ArmSceneState.showLimits; label: qsTr("Selected joint and its limits"); tone: ArmSceneState.selectionColor }
            Entry { visible: ArmSceneState.showFrames; label: qsTr("Tool and target frames (x, y, z)"); tone: "#e53935" }
            Entry { visible: ArmSceneState.guideOffered; label: qsTr("Hand guide: Ctrl + drag a link"); tone: ArmSceneState.accentColor }
            Entry { visible: ArmSceneState.dragOffered; label: qsTr("Joint drag: drag a link, Enter to move · Shift: follow"); tone: ArmSceneState.accentColor }
            Entry { visible: ArmSceneState.gizmoShown && ArmSceneState.gizmoOffered; label: qsTr("Tool gizmo: drag an arrow or a ring, Enter to move"); tone: ArmSceneState.accentColor }
            Text {
                text: qsTr("Grid %1 cm%2").arg(studio.gridSpacing).arg(ArmSceneState.orthographic ? qsTr(" · orthographic") : "")
                color: ArmSceneState.textColor
                opacity: 0.8
                font.pixelSize: 11
            }
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
        function onFrameRequested() {
            root.framePending = true
            root.frameWhenSized()
        }
        function onFocusSelectedRequested() {
            if (ArmSceneState.selectionText.length > 0)
                studio.focusOn(ArmSceneState.selectionAnchor)
        }
    }
}
