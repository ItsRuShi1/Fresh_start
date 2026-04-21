import serial
import time


class ESP32Robot:
    def __init__(self, port="COM3", baud=115200):

        # ── PHYSICAL LAYOUT ──────────────────────────────────────────────────
        self.OFFSET_TO_ROBOT = 160.0
        self.X_CENTER_MM     = 65.0
        self.MAX_X_REACH     = 65.0
        self.MAX_Y_REACH     = 100.0

        # ── Z HEIGHTS ────────────────────────────────────────────────────────
        self.SAFE_Z  = -10.0
        self.PICK_Z  = -57.0
        self.THROW_Z = -55.0    # deeper into bin — less stone above jaws = less wedging

        # ── TRASH BOX POSITION ───────────────────────────────────────────────
        self.THROW_X = 110.0
        self.THROW_Y = 110.0

        # ── TIMING ───────────────────────────────────────────────────────────
        self.GRIP_T      = 0.25   # gripper close (mechanical)
        self.RELEASE_T   = 0.2    # gripper open — fast drop
        self.CMD_GAP     = 0.02   # serial buffer gap only
        self.MOVE_DOWN_T = 0.4    # SAFE_Z → PICK_Z descent
        self.THROW_DOWN_T= 0.3    # SAFE_Z → THROW_Z descent

        # ── LEAD TIME ────────────────────────────────────────────────────────
        # Compensates for belt movement during MOVE_DOWN_T descent only.
        # Tune: arm touches behind object → increase
        #       arm touches ahead of object → decrease
        self.LEAD_TIME = 0.0

        # ── MOTOR SPEED ──────────────────────────────────────────────────────
        self.MOTOR_SPEED = 5000

        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(2)
            self._initialize_robot()
        except Exception as e:
            print(f"[!] SERIAL ERROR: {e}")
            self.ser = None

    def _initialize_robot(self):
        print("[*] Initializing robot...")
        self.send("home")
        time.sleep(3)
        self.send("release")
        time.sleep(0.3)
        self.send(f"IK[0.0,0.0,{self.SAFE_Z}]")
        time.sleep(0.5)
        print("[*] Robot ready.")

    def send(self, cmd):
        if self.ser is None:
            print(f"[!] No serial: {cmd}")
            return
        try:
            self.ser.write(f"{cmd}\n".encode('utf-8'))
            time.sleep(self.CMD_GAP)
        except Exception as e:
            print(f"[!] Send error: {e}")

    def _move(self, x, y, z, label=""):
        self.send(f"IK[{x:.2f},{y:.2f},{z:.2f}]")
        if label:
            print(f"    >> {label}  [{x:.1f}, {y:.1f}, {z:.1f}]")

    def set_belt_speed(self, steps): self.send(f"SetSpeed {steps}")
    def start_belt(self):            self.send("StartStepper")
    def stop_belt(self):             self.send("StopStepper")

    def _y_now(self, exit_time, belt_speed):
        """Object Y position right now from actual wall clock."""
        return self.OFFSET_TO_ROBOT - ((time.time() - exit_time) * belt_speed)

    def _aim_y(self, exit_time, belt_speed):
        """
        Where to aim RIGHT NOW including descent lead.
        Returns (aim_y, current_y).
        aim_y is clamped to reach envelope.
        """
        cy = self._y_now(exit_time, belt_speed)
        ay = cy - (belt_speed * self.LEAD_TIME)
        return max(-self.MAX_Y_REACH, min(self.MAX_Y_REACH, ay)), cy

    def is_catchable(self, exit_time, belt_speed):
        cy = self._y_now(exit_time, belt_speed)
        return -self.MAX_Y_REACH <= cy <= (self.MAX_Y_REACH + self.OFFSET_TO_ROBOT)

    def execute_sort(self, vision_x, exit_time, belt_speed):
        """
        Pick-and-place with live intercept tracking.

        The arm tracks the object using a nudge loop — every 150ms it
        receives a fresh IK command pointing at the object's current live
        position. Firmware interpolation blends these into smooth motion.

        Two key fixes vs previous version:
        1. Nudge loop sends target_y at the moment the object will be
           when the arm ARRIVES (not when command is sent) — stops compounding.
        2. Throw sequence timings reduced — no unnecessary stacked waits.
        """
        try:
            target_x = max(-self.MAX_X_REACH,
                           min(self.MAX_X_REACH, vision_x - self.X_CENTER_MM))

            # ── CHECK: still catchable? ───────────────────────────────────────
            ay, cy = self._aim_y(exit_time, belt_speed)
            elapsed = time.time() - exit_time
            print(f"\n[>>] Object at Y={cy:.1f}mm after {elapsed:.2f}s in queue")

            if cy < -self.MAX_Y_REACH:
                print(f"[-] SKIPPED: Y={cy:.1f}mm past reach")
                return

            # ── WAIT: not yet reachable ───────────────────────────────────────
            if cy > self.MAX_Y_REACH:
                wait_s = max(0.0, ((cy - self.MAX_Y_REACH) / belt_speed) - 0.20)
                print(f"[~] Waiting {wait_s:.2f}s for object to enter reach")
                self.send(f"IK[{target_x:.2f},{self.MAX_Y_REACH:.2f},{self.SAFE_Z:.2f}]")
                time.sleep(wait_s)

            # ── NUDGE LOOP: track object until arm is in position ─────────────
            # Each iteration:
            #   1. Calculate where object will be in NUDGE_INTERVAL seconds
            #      (i.e. where it will be when the NEXT command fires)
            #   2. Send IK to that position
            # This prevents compounding — we always aim at the FUTURE position,
            # not the current one, so the arm converges rather than overshooting.
            NUDGE_INTERVAL = 0.15   # seconds between nudge commands
            MAX_NUDGES     = 12     # max 1.8s of tracking before forcing descent

            for i in range(MAX_NUDGES):
                # Aim at where object will be NUDGE_INTERVAL seconds from now
                # (when the next command would fire if we kept looping)
                future_y = self._y_now(exit_time, belt_speed) - (belt_speed * NUDGE_INTERVAL)
                future_y = max(-self.MAX_Y_REACH, min(self.MAX_Y_REACH, future_y))
                cy       = self._y_now(exit_time, belt_speed)

                if cy < -self.MAX_Y_REACH:
                    print(f"[-] MISSED in nudge loop at iteration {i}")
                    self._move(0.0, 0.0, self.SAFE_Z, "home")
                    return

                self.send(f"IK[{target_x:.2f},{future_y:.2f},{self.SAFE_Z:.2f}]")
                print(f"    [nudge {i+1}] aim Y={future_y:.1f}mm  object Y={cy:.1f}mm")
                time.sleep(NUDGE_INTERVAL)

                # Exit nudge loop when object is within 20mm of arm's current aim
                # — arm is close enough, time to descend
                if abs(cy - future_y) < 20.0:
                    print(f"    [nudge] converged — descending")
                    break

            # ── DESCEND ───────────────────────────────────────────────────────
            # One final live recalculation at the exact moment of descent.
            ay, cy = self._aim_y(exit_time, belt_speed)
            if cy < -self.MAX_Y_REACH:
                print(f"[-] MISSED before descent"); self._move(0.0, 0.0, self.SAFE_Z, "home"); return

            print(f"    [descend] Y={ay:.1f}mm (object at Y={cy:.1f}mm)")
            self.send(f"IK[{target_x:.2f},{ay:.2f},{self.PICK_Z:.2f}]")
            time.sleep(self.MOVE_DOWN_T)

            # ── GRIP ─────────────────────────────────────────────────────────
            self.send("suck")
            print("    >> suck")
            time.sleep(self.GRIP_T)

            # ── LIFT straight up ──────────────────────────────────────────────
            self.send(f"IK[{target_x:.2f},{ay:.2f},{self.SAFE_Z:.2f}]")
            time.sleep(self.MOVE_DOWN_T)

            # ── TRAVEL to bin ─────────────────────────────────────────────────
            self._move(self.THROW_X, self.THROW_Y, self.SAFE_Z, "travel to bin")

            # ── LOWER into bin ────────────────────────────────────────────────
            self.send(f"IK[{self.THROW_X:.2f},{self.THROW_Y:.2f},{self.THROW_Z:.2f}]")
            time.sleep(self.THROW_DOWN_T)   # wait until arm fully at bin depth

            # ── RELEASE SEQUENCE — shake to break friction wedge ─────────────
            # Large stones get wedged between parallel jaws — simple release
            # does not generate enough force to overcome friction.
            # Solution: rapid grip/release pulses create micro-vibration that
            # breaks the wedge, then a final held release drops the stone.

            # Phase 1: Initial release attempt
            self.send("release")
            print("    >> release attempt")
            time.sleep(self.RELEASE_T)

            # Phase 2: Shake sequence — 4 rapid grip/release pulses
            # Each pulse briefly re-grips then immediately releases.
            # The alternating force breaks the friction wedge on large stones.
            print("    >> shake sequence")
            for i in range(4):
                self.send("suck")
                time.sleep(0.05)           # faster grip pulse
                self.send("release")
                time.sleep(0.06)           # faster release pulse
            print("    >> shake done")

            # Phase 3: Final held release — stone should now be free
            self.send("release")
            time.sleep(self.RELEASE_T)     # full open time — stone drops
            print("    >> object in bin")

            # Phase 4: Keep gripper open while lifting.
            # Send release just before IK so firmware cannot re-close it.
            self.send("release")
            time.sleep(0.05)
            self.send(f"IK[{self.THROW_X:.2f},{self.THROW_Y:.2f},{self.SAFE_Z:.2f}]")
            print("    >> lift — gripper open")
            time.sleep(self.MOVE_DOWN_T)

            # ── RETURN HOME ───────────────────────────────────────────────────
            self._move(0.0, 0.0, self.SAFE_Z, "return home")
            print("[+] Cycle complete.\n")

        except Exception as e:
            print(f"[!] execute_sort error: {e}")
            try:
                self.send(f"IK[0.00,0.00,{self.SAFE_Z}]")
            except Exception:
                pass

    def close(self):
        self.stop_belt()
        self.send("release")
        self.send("home")
        if self.ser:
            self.ser.close()