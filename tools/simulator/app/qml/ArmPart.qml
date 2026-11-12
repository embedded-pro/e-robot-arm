import QtQuick
import QtQuick3D

Model {
    property int joint: -1
    property bool ghost: false

    pickable: !ghost
    castsShadows: !ghost
    receivesShadows: !ghost
}
