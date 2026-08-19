#!/usr/bin/env python3
"""
Touchscreen-safe frameless dialog base.

The PFD runs on a touchscreen with no window decorations, so a dialog that
slips behind the main window is effectively lost — there is no title bar to
grab and no taskbar to recover it from. TouchSafeDialog keeps every pop-up
reachable no matter where the user touches.
"""

from PyQt6.QtWidgets import QDialog, QApplication, QWidget
from PyQt6.QtCore import Qt, QEvent


class TouchSafeDialog(QDialog):
    """Frameless dialog that cannot be buried by the main window.

    Three measures, because no single one is sufficient on its own:

      * ``WindowStaysOnTopHint`` asks the window manager to keep the dialog
        above the main window in the stacking order.
      * The dialog is centered over its parent window when shown, so on a
        large panel it always appears within reach rather than off in a
        corner.
      * While visible, it watches the application for presses that land
        outside itself and re-raises. This is the belt-and-braces part: some
        window managers (and macOS in fullscreen) will still promote the
        main window on click despite the stay-on-top hint.

    The event filter is installed only while the dialog is open, so nothing
    of this runs during the normal 30 Hz refresh, and it returns immediately
    for every event type other than a press.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowFlags(
            Qt.WindowType.FramelessWindowHint
            | Qt.WindowType.Dialog
            | Qt.WindowType.WindowStaysOnTopHint
        )
        self._raise_filter_active = False

    # ─── Lifecycle ───

    def showEvent(self, event):
        super().showEvent(event)
        self._center_on_parent()
        self.raise_()
        self.activateWindow()
        if not self._raise_filter_active:
            app = QApplication.instance()
            if app is not None:
                app.installEventFilter(self)
                self._raise_filter_active = True

    def hideEvent(self, event):
        self._remove_filter()
        super().hideEvent(event)

    def closeEvent(self, event):
        self._remove_filter()
        super().closeEvent(event)

    def _remove_filter(self):
        if self._raise_filter_active:
            app = QApplication.instance()
            if app is not None:
                app.removeEventFilter(self)
            self._raise_filter_active = False

    # ─── Placement ───

    def _center_on_parent(self):
        """Center over the parent window, clamped so an oversized dialog
        stays anchored to the top-left of the panel rather than hanging off
        it."""
        # Settle the layout first — on the first show a dialog sized by its
        # contents has no final geometry yet.
        layout = self.layout()
        if layout is not None:
            layout.activate()

        parent = self.parentWidget()
        if parent is not None:
            ref = parent.window().frameGeometry()
        else:
            screen = QApplication.primaryScreen()
            if screen is None:
                return
            ref = screen.availableGeometry()

        x = ref.x() + (ref.width() - self.width()) // 2
        y = ref.y() + (ref.height() - self.height()) // 2
        self.move(max(ref.x(), x), max(ref.y(), y))

    # ─── Stay-on-top enforcement ───

    def eventFilter(self, obj, event):
        if event.type() in (QEvent.Type.MouseButtonPress,
                            QEvent.Type.TouchBegin):
            if isinstance(obj, QWidget) and not self._owns(obj):
                self.raise_()
        return super().eventFilter(obj, event)

    def _owns(self, widget: QWidget) -> bool:
        """True if the widget belongs to this dialog — either a child widget
        or a window the dialog spawned.

        The parent chain, rather than isAncestorOf(), is what makes combo box
        drop-downs and nested message boxes work: those are separate
        top-level windows parented to a widget inside the dialog, and
        re-raising over them would dismiss the very list the user is trying
        to touch.
        """
        node = widget
        while node is not None:
            if node is self:
                return True
            node = node.parent()
        return False
