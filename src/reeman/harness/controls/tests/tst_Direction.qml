import QtQuick
import QtTest
import ".." as Controls

Item {
    width: 300
    height: 240

    QtObject {
        id: receiver
        property var commands: []
        function command(direction) { commands.push(direction) }
    }

    Controls.ReemanControls {
        id: panel
        anchors.fill: parent
        controller: receiver
    }

    TestCase {
        name: "DirectionSelection"
        when: windowShown

        function init() {
            panel.stop()
            panel.forceActiveFocus()
            receiver.commands = []
        }

        function test_turnReplacesForward() {
            keyPress(Qt.Key_W)
            compare(panel.direction, 1)
            keyPress(Qt.Key_A)
            compare(panel.direction, 3)
            compare(receiver.commands.slice(-1)[0], 3)
            keyRelease(Qt.Key_W)
            compare(panel.direction, 3)
            keyPress(Qt.Key_D)
            keyRelease(Qt.Key_A)
            compare(panel.direction, 4)
            keyRelease(Qt.Key_D)
            tryCompare(panel, "direction", 0, 700)
            compare(receiver.commands.slice(-1)[0], 0)
        }

        function test_stopCancelsHeldDirection() {
            keyPress(Qt.Key_W)
            compare(panel.direction, 1)
            keyClick(Qt.Key_Space)
            compare(panel.direction, 0)
            keyRelease(Qt.Key_W)
            wait(300)
            compare(receiver.commands.slice(-1)[0], 0)
        }
    }
}