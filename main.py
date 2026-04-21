import cv2
import threading
import queue
import time
from vision_engine import VisionSystem
from robot_controller import ESP32Robot
from tracker_storage import ObjectTracker

# --- CONFIG ---
# Physical belt speed in mm/s — calibrate with: distance_mm / travel_time_s
BELT_SPEED_MM_S = 21.0


MODEL_PATH = "C:/Users/palla/Robotic_hand/robotic_hand_simulation/model/best.pt"

# --- INIT ---
tracker     = ObjectTracker()
target_queue = queue.Queue()
ids_lock    = threading.Lock()
queued_ids  = set()   # IDs already handed to the robot queue

eye  = VisionSystem(model_path=MODEL_PATH)
hand = ESP32Robot(port="COM3")

hand.set_belt_speed(5000)   # steps/s — must match BELT_SPEED_MM_S physically
hand.start_belt()


def robot_worker():
    """
    Processes pick tasks one at a time from the queue.
    Each task is triggered only after the object has exited the vision box.
    """
    while True:
        track_id = target_queue.get()
        try:
            if track_id is None:
                break

            item = tracker.get_data(track_id)
            if not item:
                print(f"[!] No data for ID:{track_id}")
                continue

            if 'exit_time' not in item:
                print(f"[!] Missing exit_time for ID:{track_id}, skipping")
                continue

            print(f"[*] Task Start: {item['label']} (ID:{track_id})")

            # Check if object is still catchable before starting the pick.
            # If arm was busy with a previous object, this one may have already
            # passed the reach zone — skip it rather than attempting a bad pick.
            if not hand.is_catchable(item['exit_time'], BELT_SPEED_MM_S):
                print(f"[!] SKIPPED ID:{track_id} — already past reach zone. "
                      f"Space objects further apart on the belt.")
            else:
                hand.execute_sort(
                    vision_x=item['robot_x'],
                    exit_time=item['exit_time'],
                    belt_speed=BELT_SPEED_MM_S
                )

            tracker.mark_done(track_id)
            tracker.cleanup()

        except Exception as e:
            print(f"[!] robot_worker error: {e}")
        finally:
            target_queue.task_done()   # always release, even on error


threading.Thread(target=robot_worker, daemon=True).start()

cap = cv2.VideoCapture(0)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.rotate(frame, cv2.ROTATE_180)

        # get_targets() now returns ONLY objects that just exited the vision box
        # Objects still inside the box are tracked silently — NO robot reaction
        newly_exited, debug_frame = eye.get_targets(frame)

        for obj in newly_exited:
            with ids_lock:
                if obj['id'] in queued_ids:
                    continue
                queued_ids.add(obj['id'])

            # Store the object data — crucially using exit_time, not detect_time
            tracker.add_object(
                obj_id=obj['id'],
                label=obj['label'],
                robot_x=obj['robot_x'],
                robot_y=obj['robot_y'],
                exit_time=obj['exit_time'],   # moment it left the vision box
            )

            # Queue the pick task — robot_worker will handle it
            target_queue.put(obj['id'])
            print(f"[>>] Queued pick for ID:{obj['id']} ({obj['label']}) "
                  f"at X={obj['robot_x']:.1f}mm")

        cv2.imshow("Multi-Object Sorter", debug_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    target_queue.put(None)
    hand.close()
    cap.release()
    cv2.destroyAllWindows()
