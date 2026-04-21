import threading


class ObjectTracker:
    """
    Thread-safe store for objects that have exited the vision box.

    Key change: we now store exit_time (when object left the vision box)
    instead of detect_time (when it first appeared). The robot uses exit_time
    to calculate how far the object has travelled since leaving the camera zone.
    """

    def __init__(self):
        self._data = {}
        self._lock = threading.Lock()

    def add_object(self, obj_id, label, robot_x, robot_y, exit_time):
        """
        Store an object that has just exited the vision box.

        Args:
            obj_id    (int):   tracker ID
            label     (str):   class name
            robot_x   (float): X position in mm when last seen in vision box
            robot_y   (float): Y position in mm when last seen (≈ EXIT_Y_THRESHOLD)
            exit_time (float): time.time() when object left the vision box
        """
        with self._lock:
            self._data[obj_id] = {
                'label':     label,
                'robot_x':   robot_x,
                'robot_y':   robot_y,
                'exit_time': exit_time,
                'done':      False,
            }

    def get_data(self, obj_id):
        with self._lock:
            return self._data.get(obj_id, None)

    def mark_done(self, obj_id):
        with self._lock:
            if obj_id in self._data:
                self._data[obj_id]['done'] = True

    def cleanup(self, max_size=200):
        with self._lock:
            done_ids = [k for k, v in self._data.items() if v['done']]
            if len(self._data) > max_size:
                for k in done_ids:
                    del self._data[k]