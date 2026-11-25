import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Window

FocusScope {
    id: controls
    Layout.minimumWidth: 235
    Layout.minimumHeight: 185
    focus: true

    property var controller: _ReemanControls
    property int direction: 0
    property bool releasePending: false

    function drive(nextDirection) {
        forceActiveFocus()
        direction = nextDirection
        releasePending = false
        controller.command(direction)
        minimumPulse.restart()
    }

    function release(previousDirection) {
        if (direction !== previousDirection)
            return
        if (minimumPulse.running)
            releasePending = true
        else
            stop()
    }

    function stop() {
        if (direction === 0)
            return
        direction = 0
        releasePending = false
        minimumPulse.stop()
        controller.command(0)
    }

    function directionForKey(key) {
        switch (key) {
        case Qt.Key_W: case Qt.Key_Up: return 1
        case Qt.Key_S: case Qt.Key_Down: return 2
        case Qt.Key_A: case Qt.Key_Left: return 3
        case Qt.Key_D: case Qt.Key_Right: return 4
        default: return 0
        }
    }

    Keys.onPressed: function(event) {
        const selected = directionForKey(event.key)
        if (selected) {
            if (!event.isAutoRepeat)
                drive(selected)
            event.accepted = true
        } else if (event.key === Qt.Key_Space) {
            stop()
            event.accepted = true
        }
    }
    Keys.onReleased: function(event) {
        const selected = directionForKey(event.key)
        if (selected) {
            if (!event.isAutoRepeat)
                release(selected)
            event.accepted = true
        }
    }

    onVisibleChanged: if (!visible) stop()
    onActiveFocusChanged: if (!activeFocus) stop()
    Connections {
        target: controls.Window.window
        function onActiveChanged() {
            if (!target.active)
                controls.stop()
        }
    }

    Timer {
        id: minimumPulse
        interval: 250
        onTriggered: if (controls.releasePending) controls.stop()
    }

    Timer {
        interval: 100
        repeat: true
        running: controls.direction !== 0
        onTriggered: controls.controller.command(controls.direction)
    }

    GridLayout {
        anchors.centerIn: parent
        columns: 3
        rowSpacing: 5
        columnSpacing: 5
        Repeater {
            model: [
                { label: "▲", row: 0, column: 1, direction: 1 },
                { label: "◀", row: 1, column: 0, direction: 3 },
                { label: "Stop", row: 1, column: 1, direction: 0 },
                { label: "▶", row: 1, column: 2, direction: 4 },
                { label: "▼", row: 2, column: 1, direction: 2 }
            ]
            delegate: Button {
                required property var modelData
                text: modelData.label
                focusPolicy: Qt.NoFocus
                highlighted: modelData.direction !== 0 && controls.direction === modelData.direction
                Layout.row: modelData.row
                Layout.column: modelData.column
                Layout.preferredWidth: 65
                onPressed: modelData.direction ? controls.drive(modelData.direction) : controls.stop()
                onReleased: if (modelData.direction) controls.release(modelData.direction)
                onCanceled: if (controls.direction === modelData.direction) controls.stop()
            }
        }
    }
}
