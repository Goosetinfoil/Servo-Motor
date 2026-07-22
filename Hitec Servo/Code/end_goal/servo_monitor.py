import json
import math
import os
import sys
import tkinter as tk
from tkinter import ttk
import servo_classes


# Links the app to save_profiles.json so that it can save and load the servo profiles
if getattr(sys, "frozen", False):  
    APP_DIR = os.path.dirname(os.path.abspath(sys.executable))
else:
    APP_DIR = os.path.dirname(os.path.abspath(__file__))

PROFILES_FILE = os.path.join(APP_DIR, "servo_profiles.json")


# Functuon to have '-' in the boxes to write values
def set_entry(entry, value, placeholder="-"):
        entry.delete(0, tk.END)
        entry.insert(0, str(value) if value is not None else placeholder)


# ----------------------------------------
#  Profiles for Hitec Servo
# ----------------------------------------

# When a new profile is created, these are the default values that will be saved to profile.
DEFAULT_PROFILE_SETTINGS = {
    "speed": 1000,
    "pos_max": 10922,
    "pos_min": 5462,
    "pos_mid": 8192,
    "stream_time": 1000,
    "speed_up": 0,
    "speed_dn": 0,
    "sample_point": 1,
    "baud_rate": 0,
    "stream_mode": 0,
    "id1": 1,
    "id2": 0,
}


POSITION_COORDS = {   
    "safety_tether": (389, 636),   # placeholder - actually computed from
    "emergency_vent": (326, 681),  # TETHER_ANGLE_DEG / VENT_ANGLE_DEG at runtime
}

# Spot on the vehicle diagram a profile name refers to by looking for keywords in the name. Returns key into POSITION_COORDS
def match_profile_to_position(profile_name):
    """Determines which spot on the vehicle diagram a profile belongs to
    purely from keywords in its name. Returns a key into POSITION_COORDS """

    name = profile_name.lower()

    if "tether" in name:
        return "safety_tether"
    if "vent" in name:
        return "emergency_vent"

    vertical = "top" if "top" in name else ("bottom" if "bottom" in name else None)
    horizontal = "left" if "left" in name else ("right" if "right" in name else None)
    depth = "inner" if "inner" in name else ("outer" if "outer" in name else None)

    if vertical and horizontal and depth:
        key = f"{vertical}_{horizontal}_{depth}"
        if key in POSITION_COORDS:
            return key
    return None

# This loads up the .json file in the dropdown menu. If the file doesn't exist, it returns an empty dict. 
def load_profiles():
    """Reads servo_profiles.json from disk and returns it as a dict of
    {profile_name: settings_dict}. If the file doesn't exist yet (e.g.
    first run), returns an empty dict instead of raising an error."""
    if not os.path.exists(PROFILES_FILE):
        return {}
    with open(PROFILES_FILE, "r") as f:
        return json.load(f)

# This converts the dict to JSON text and writes it in a readable form if open directly
def save_profiles(profiles):
    with open(PROFILES_FILE, "w") as f:
        json.dump(profiles, f, indent=2)


def build_cubemars_tab(parent, cubemars):
    """Builds every widget on the CubeMars tab (labels, entry fields for
    controller ID / CAN baud / MCCONF values, Read All / Write All
    buttons) and wires them up to the cubemars_* protocol functions.
    `parent` is the tkinter Frame for this tab. Called once from main();
    doesn't return anything, just populates `parent` in place."""
    inner = tk.Frame(parent, width=640, height=830)
    inner.pack_propagate(False)
    inner.place(relx=0.5, rely=0, anchor="n")

    status_var = tk.StringVar(value="Not connected.")
    id_var = tk.StringVar(value="Controller ID: -")
    baud_var = tk.StringVar(value="CAN Baud: -")

    cubemars_state = {"connected": False}

    def poll_cubemars():
        """Runs every second (via parent.after) to check whether the
        CubeMars adapter is plugged in, and updates the status label
        when that connected/disconnected state changes. Just a USB
        presence check - no serial communication with the servo itself."""
        port = cubemars.find_port()
        if port and not cubemars_state["connected"]:
            cubemars_state["connected"] = True
            status_var.set(f"Servo detected on {port}. Click Read All.")
        elif not port and cubemars_state["connected"]:
            cubemars_state["connected"] = False
            status_var.set("Not connected.")
        parent.after(1000, poll_cubemars)

    poll_cubemars()

    tk.Label(inner, text="CubeMars AK80-8", font=("Segoe UI", 16, "bold")).pack(pady=(14, 6))
    tk.Label(inner, textvariable=id_var, font=("Segoe UI", 14)).pack(pady=1)
    tk.Label(inner, textvariable=baud_var, font=("Segoe UI", 14)).pack(pady=1)

    tk.Label(inner, text="New Controller ID (0-255):", font=("Segoe UI", 12)).pack(pady=(10, 0))
    new_id_entry = tk.Entry(inner, width=10, font=("Segoe UI", 12))
    new_id_entry.pack(pady=(2, 10))

    tk.Label(inner, text="New CAN Baud Rate:", font=("Segoe UI", 12)).pack()
    new_baud_var = tk.StringVar(value="-")
    baud_box = ttk.Combobox(inner, textvariable=new_baud_var, state="readonly",
                            values=["-"] + list(servo_classes.CubeMarsServo.CUBEMARS_CAN_BAUD_LABELS.values()), width=10, font=("Segoe UI", 12))
    baud_box.pack(pady=(2, 14))

    tk.Label(inner, text="Motor Config", font=("Segoe UI", 14, "bold")).pack(pady=(10, 6))

    limits_frame = tk.Frame(inner)
    limits_frame.pack(pady=(0, 14))

    mcconf_entries = {}

    limit_pairs = [
        ("motor_max", "motor_min"),
        ("batt_max", "batt_min"),
        ("max_erpm", "min_erpm"),
    ]

    for row, (left_field, right_field) in enumerate(limit_pairs):
        tk.Label(limits_frame, text=servo_classes.CubeMarsServo.CUBEMARS_DISPLAY_NAMES[left_field], font=("Segoe UI", 12)).grid(
            row=row, column=0, sticky="w", padx=(0, 4), pady=4)
        left_entry = tk.Entry(limits_frame, width=15, font=("Segoe UI", 12))
        left_entry.grid(row=row, column=1, padx=(0, 20), pady=2)
        mcconf_entries[left_field] = left_entry

        tk.Label(limits_frame, text=servo_classes.CubeMarsServo.CUBEMARS_DISPLAY_NAMES[right_field], font=("Segoe UI", 12)).grid(
            row=row, column=2, sticky="w", padx=(0, 4), pady=4)
        right_entry = tk.Entry(limits_frame, width=15, font=("Segoe UI", 12))
        right_entry.grid(row=row, column=3, pady=2)
        mcconf_entries[right_field] = right_entry

    tk.Label(inner, text="Speed", font=("Segoe UI", 13, "bold")).pack(pady=(4, 4))
    speed_frame = tk.Frame(inner)
    speed_frame.pack(pady=(0, 10))

    for col, field in enumerate(["speed_kp", "speed_ki"]):
        short_label = servo_classes.CubeMarsServo.CUBEMARS_DISPLAY_NAMES[field].replace("Speed ", "")
        tk.Label(speed_frame, text=short_label, font=("Segoe UI", 12)).grid(
            row=0, column=col * 2, sticky="w", padx=(0 if col == 0 else 20, 4))
        entry = tk.Entry(speed_frame, width=15, font=("Segoe UI", 12))
        entry.grid(row=0, column=col * 2 + 1)
        mcconf_entries[field] = entry

    tk.Label(inner, text="Position", font=("Segoe UI", 13, "bold")).pack(pady=(4, 4))
    position_frame = tk.Frame(inner)
    position_frame.pack(pady=(0, 14))

    for col, field in enumerate(["position_kp", "position_ki", "position_kd"]):
        short_label = servo_classes.CubeMarsServo.CUBEMARS_DISPLAY_NAMES[field].replace("Position ", "")
        tk.Label(position_frame, text=short_label, font=("Segoe UI", 12)).grid(
            row=0, column=col * 2, sticky="w", padx=(0 if col == 0 else 20, 4))
        entry = tk.Entry(position_frame, width=15, font=("Segoe UI", 12))
        entry.grid(row=0, column=col * 2 + 1)
        mcconf_entries[field] = entry

    def on_read_all():
        """Handler for the 'Read All' button: reads app config (for
        controller ID and CAN baud) and motor config (for the limit/PID
        entry fields), and pushes every value into its widget. Bails out
        and shows the error in status_var if either read fails."""
        status_var.set("Reading...")
        parent.update()

        app_result, app_error = cubemars.cubemars_read_appconf()
        if app_error:
            status_var.set(app_error)
            return
        id_var.set(f"Controller ID: {app_result[5]}")
        baud_var.set(f"CAN Baud: {servo_classes.CubeMarsServo.CUBEMARS_CAN_BAUD_LABELS.get(app_result[17], 'unknown')}")

        mc_result, mc_error = cubemars.cubemars_read_mcconf()
        if mc_error:
            status_var.set(mc_error)
            return
        for field_name, entry in mcconf_entries.items():
            set_entry(entry, servo_classes.CubeMarsServo.cubemars_get_mcconf_field(mc_result, field_name))

        status_var.set("Read complete.")

    def on_write_all():
        """Handler for the 'Write All' button: validates every non-empty
        field (controller ID 0-255, a chosen baud label, and any MCCONF
        numbers typed in), then only sends the writes that are actually
        needed - app config and/or motor config - skipping calls
        entirely if nothing changed in that group. Refreshes the display
        with the servo's confirmed values after each write."""
        id_text = new_id_entry.get().strip()
        new_id = None
        if id_text:
            try:
                candidate = int(id_text)
                if 0 <= candidate <= 255:
                    new_id = candidate
                else:
                    status_var.set(f"{candidate} is out of range (0-255). ID not changed.")
                    return
            except ValueError:
                status_var.set(f"'{id_text}' is not a number. ID not changed.")
                return

        new_baud = None
        chosen_label = new_baud_var.get()
        if chosen_label != "-":
            for value, label in servo_classes.CubeMarsServo.CUBEMARS_CAN_BAUD_LABELS.items():
                if label == chosen_label:
                    new_baud = value
                    break

        changes = {}
        for field_name, entry in mcconf_entries.items():
            text = entry.get().strip()
            if text:
                try:
                    changes[field_name] = float(text)
                except ValueError:
                    status_var.set(f"'{text}' is not a valid number for {field_name}.")
                    return
        
        if new_id is None and new_baud is None and not changes:
            status_var.set("No values entered - nothing to write.")
            return

        status_var.set("Writing...")
        parent.update()

        wrote_something = False

        if new_id is not None or new_baud is not None:
            app_result, app_error = cubemars.cubemars_write_appconf(new_id, new_baud)
            if app_error:
                status_var.set(app_error)
                return
            id_var.set(f"Controller ID: {app_result[5]}")
            baud_var.set(f"CAN Baud: {servo_classes.CubeMarsServo.CUBEMARS_CAN_BAUD_LABELS.get(app_result[17], 'unknown')}")
            wrote_something = True

        if changes:
            mc_result, mc_error = cubemars.cubemars_write_mcconf(changes)
            if mc_error:
                status_var.set(mc_error)
                return
            for field_name, entry in mcconf_entries.items():
                            set_entry(entry, servo_classes.CubeMarsServo.cubemars_get_mcconf_field(mc_result, field_name))
            wrote_something = True

        if wrote_something:
            status_var.set("Written and verified.")

    tk.Button(inner, text="Read All", command=on_read_all,
             font=("Segoe UI", 12), width=18).pack(pady=(6, 4))
    tk.Button(inner, text="Write All", command=on_write_all,
             font=("Segoe UI", 12), width=18).pack(pady=(0, 14))

    tk.Label(inner, textvariable=status_var, fg="gray", font=("Segoe UI", 14),
             wraplength=500, justify="center").pack(pady=(10, 8))


# -----------------------------------------------
#  GUI
# -----------------------------------------------


def main():
    """Entry point. Builds the main window and the two-tab Notebook,
    builds every widget and handler for the Hitec tab directly inline
    (IDs, profile selector, vehicle diagram, settings fields, save
    buttons), delegates the CubeMars tab to build_cubemars_tab(), starts
    the two background polling loops (poll(), poll_global_status()),
    and finally hands control to tkinter's mainloop(). Doesn't return
    until the window is closed."""
    hitec = servo_classes.HitecServo()
    cubemars = servo_classes.CubeMarsServo()
    root = tk.Tk() # Creates a new window
    root.title("Servo Monitor") # Title of the GUI
    root.geometry("1250x820") # Size of the GUI window

    root.option_add("*TCombobox*Listbox.font", ("Segoe UI", 11)) # Font size for the text in the dropdown menu of the profile selector

    global_status_var = tk.StringVar(value="No servo connected.")
    tk.Label(root, textvariable=global_status_var, font=("Segoe UI", 11, "bold"),
             fg="#555555").pack(side=tk.TOP, fill=tk.X, pady=(4, 0))

    notebook = ttk.Notebook(root)
    notebook.pack(fill=tk.BOTH, expand = True)

    hitec_tab = tk.Frame(notebook)
    cubemars_tab = tk.Frame(notebook)
    notebook.add(hitec_tab, text="Hitec")
    notebook.add(cubemars_tab, text="CubeMars")

    style = ttk.Style()
    style.configure("TNotebook.Tab", font=("Segoe UI", 13), padding=(20,8))

    def on_tab_change(event):
        selected = event.widget.select()
        tab_text = event.widget.tab(selected, "text")
        if tab_text == "CubeMars":
            root.geometry("620x840")
        else:
            root.geometry("1250x860")
    
    notebook.bind("<<NotebookTabChanged>>", on_tab_change)    
        

    # Creating two columns in the window, content_frame for the parameters and diagram_panel for the airships visual
    content_frame = tk.Frame(hitec_tab)
    content_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10, pady=10)
    diagram_panel = tk.Frame(hitec_tab)
    diagram_panel.pack(side=tk.RIGHT, fill=tk.BOTH, padx=10, pady=10)


    status_var = tk.StringVar(value="Waiting for servo...") # The default status when code run

    # Creates a label for ID1 and ID2
    id1_var = tk.StringVar(value="ID1 (Actuator ID): -")
    id2_var = tk.StringVar(value="ID2 (CAN Node ID): -")

    # Creates a label for Servo IDs and the actual ID's when button is pressed
    tk.Label(content_frame, text="Servo IDs", font=("Segoe UI", 16, "bold")).pack(pady=(14, 6))
    tk.Label(content_frame, textvariable=id1_var, font=("Segoe UI", 14)).pack(pady=1)
    tk.Label(content_frame, textvariable=id2_var, font=("Segoe UI", 14)).pack(pady=1)

    def on_read_id(): # This function is giving information as soon as the Read ID button is pressed 
        """Handler for the 'Read ID' button: calls read_ids_only(),
        updates the ID1/ID2 labels with whatever came back (or 'read
        failed' per-ID if a value was None), and updates the status
        line."""
        status_var.set("Reading ID...")
        root.update()
        ids, error = hitec.read_ids_only()
        if error:
            status_var.set(error)
            return
        id1, id2 = ids
        id1_var.set(f"ID1 (Actuator ID): {id1 if id1 is not None else 'read failed'}")
        id2_var.set(f"ID2 (CAN Node ID): {id2 if id2 is not None else 'read failed'}")
        status_var.set("ID read.")

    tk.Button(content_frame, text="Read ID", command=on_read_id,
             font=("Segoe UI", 12), width=12).pack(pady=(2, 4)) # This is the code to create the Read ID button 


    tk.Label(content_frame, text="Servo Profile", font=("Segoe UI", 16, "bold")).pack(pady=(14, 4)) # A label to indicate the Servo Profiles

    profile_frame = tk.Frame(content_frame) 
    profile_frame.pack()

    profiles = load_profiles() # Creating a variable called profiles and assigning it the profile dict
    profile_var = tk.StringVar(value="-")
    profile_box = ttk.Combobox(profile_frame, textvariable=profile_var,
                               values=list(profiles.keys()), width=44, state="readonly", font =("Segoe UI", 11))
    profile_box.grid(row=0, column=0, columnspan=3, padx=4, pady=(0, 6)) # The placement of the box 

    new_profile_entry = tk.Entry(profile_frame, width=24, font = ("Segoe UI", 11))
    new_profile_entry.grid(row=1, column=0, padx=4)

    tk.Button(profile_frame, text="Add", width=6, font=("Segoe UI", 11),
             command=lambda: on_add_profile()).grid(row=1, column=1, padx=4) # Button to add a profile 
    tk.Button(profile_frame, text="Delete", width=7, font=("Segoe UI", 11),
             command=lambda: on_delete_profile()).grid(row=1, column=2, padx=4) # Button to delete a profile

    tk.Button(content_frame, text="Save to profile only", command=lambda: on_save_to_profile(),
             font=("Segoe UI", 13), width=20).pack(pady=(6, 4)) # Button to save the setting to the servo only

    # --- Vehicle diagram (always visible, embedded in the main window) ---
    # Drawn directly with Canvas shapes (no external image file), so it
    # looks like a native part of the app rather than a pasted-in photo.
    # Because WE define this coordinate system, the inner/outer marker
    # positions below are calculated exactly rather than estimated off
    # a picture - change BODY_R / TIP_R / TICK_R to reshape the whole
    # diagram if the real vehicle's proportions differ.
    diagram_state = {"canvas": None, "marker": None}

    BACK_CENTER = (240, 250)   # back-view circle center
    BODY_R = 90                   # back-view body radius
    TIP_R = 200                   # arm length (tip distance from center)
    TICK_R = 138   # where the inner/outer tick mark sits
    ARM_WIDTH = 30

    SIDE_CENTER = (240, 600)      # side-view ellipse center
    SIDE_RX, SIDE_RY = 180, 80 # Size of the side view oval 

    # Angles (degrees, 0=right/east, increasing counter-clockwise) where
    # the vent and tether sit on the side-view ellipse's boundary - this
    # is what makes them look physically attached rather than floating.
    # Changing these numbers is how you move the vent/tether around the
    # ellipse - they're used to CALCULATE the coordinates below, which
    # then get written into POSITION_COORDS so the drawn shape and the
    # red marker dot always agree on where these two actually are.
    VENT_ANGLE_DEG = 35 # Position of the vent drawing
    TETHER_ANGLE_DEG = 8 # Position of the tether drawing 

    def _point_on_ellipse(center, rx, ry, angle_deg):
        """Converts a polar angle on an ellipse (center, radii rx/ry)
        into (x, y) canvas coordinates on that ellipse's boundary. Used
        to place the vent/tether markers exactly on the drawn hull line
        rather than guessing pixel coordinates by eye."""
        rad = math.radians(angle_deg)
        cx, cy = center
        return (cx + rx * math.cos(rad), cy + ry * math.sin(rad))

    POSITION_COORDS["emergency_vent"] = _point_on_ellipse(
        SIDE_CENTER, SIDE_RX, SIDE_RY, VENT_ANGLE_DEG)
    POSITION_COORDS["safety_tether"] = _point_on_ellipse(
        SIDE_CENTER, SIDE_RX, SIDE_RY, TETHER_ANGLE_DEG)

    ARM_DIRECTIONS = {
        # unit vectors pointing from the body center toward each arm
        "top_left": (-0.7071, -0.7071),
        "top_right": (0.7071, -0.7071),
        "bottom_left": (-0.7071, 0.7071),
        "bottom_right": (0.7071, 0.7071),
    }

    def _point_along_arm(direction, radius):
        """Given a unit vector `direction` (from ARM_DIRECTIONS) and a
        distance `radius` from the body center, returns the (x, y)
        canvas point that far out along that arm. Used to compute the
        inner/outer marker positions for each of the 4 arms."""
        dx, dy = direction
        cx, cy = BACK_CENTER

        return (cx + radius * dx, cy + radius * dy)
    
    for name, direction in ARM_DIRECTIONS.items():
        inner_radius = (BODY_R + TICK_R) / 2   # midpoint of the inner segment
        outer_radius = (TICK_R + TIP_R) / 2    # midpoint of the outer segment
        POSITION_COORDS[f"{name}_inner"] = _point_along_arm(direction, inner_radius)
        POSITION_COORDS[f"{name}_outer"] = _point_along_arm(direction, outer_radius)

    def update_diagram_marker():
        """Redraws the green dot on the vehicle diagram to match
        whichever profile is currently selected. Removes any existing
        marker first, then (if the selected profile's name maps to a
        known position via match_profile_to_position) draws a new one
        there. Called whenever the selected profile changes, or when a
        profile is deleted / the servo disconnects."""
        canvas = diagram_state.get("canvas")
        if canvas is None:
            return

        if diagram_state.get("marker") is not None:
            canvas.delete(diagram_state["marker"])
            diagram_state["marker"] = None

        name = profile_var.get()
        if name and name != "-":
            key = match_profile_to_position(name)
            if key:
                x, y = POSITION_COORDS[key]
                if key in ("safety_tether", "emergency_vent"): # Changes the size of the marker based on which profile is selected
                    r = 5.5
                else:
                    r = 8
                diagram_state["marker"] = canvas.create_oval(
                    x - r, y - r, x + r, y + r,
                    fill="green",outline ="green", width=1) # The fill of the marker

    def build_diagram_canvas(parent):
        """Draws the static vehicle diagram once: a back view (body
        circle + 4 rotated-rectangle arms with inner/outer tick marks)
        and a side view (hull ellipse + emergency vent grille + safety
        tether rectangle). Everything is drawn with Canvas primitives
        using the geometry constants defined above (BACK_CENTER, BODY_R,
        etc.) - there's no external image file. Stores the canvas in
        diagram_state so update_diagram_marker() can add/remove the
        selection dot on top of it later."""
        canvas = tk.Canvas(parent, width=480, height=780, bg="#f0f0f0",
                          highlightthickness=1, highlightbackground="#f0f0f0")
        canvas.pack(pady=(4, 10))
        diagram_state["canvas"] = canvas

        cx, cy = BACK_CENTER

        canvas.create_text(cx, 50, text="Back View",
                          font=("Segoe UI", 14, "italic"), fill="#000000")

        # --- back view: body + 4 arms (true rotated rectangles) with a
        #     tick mark on each showing the inner/outer split ---
        for dx, dy in ARM_DIRECTIONS.values():
            perp_dx, perp_dy = -dy, dx
            hw = ARM_WIDTH / 2
            p1x, p1y = cx + BODY_R * dx, cy + BODY_R * dy   # arm base (at body)
            p2x, p2y = cx + TIP_R * dx, cy + TIP_R * dy     # arm tip

            corners = [
                p1x + perp_dx * hw, p1y + perp_dy * hw,
                p2x + perp_dx * hw, p2y + perp_dy * hw,
                p2x - perp_dx * hw, p2y - perp_dy * hw,
                p1x - perp_dx * hw, p1y - perp_dy * hw,
            ]
            canvas.create_polygon(corners, outline="#333333", width=2, fill="#f2f2f2")

            # tick mark: short perpendicular line at the inner/outer split
            tx, ty = cx + TICK_R * dx, cy + TICK_R * dy
            half = hw
            canvas.create_line(tx - perp_dx * half, ty - perp_dy * half,
                               tx + perp_dx * half, ty + perp_dy * half,
                               width=2, fill="#333333")

        canvas.create_oval(cx - BODY_R, cy - BODY_R, cx + BODY_R, cy + BODY_R,
                          outline="#333333", width=3, fill="#f0f0f0")

        # --- side view: body ellipse + emergency vent + safety tether,
        #     both positioned ON the ellipse boundary so they read as
        #     physically attached rather than floating nearby ---
        ex, ey = SIDE_CENTER
        canvas.create_text(ex, ey - SIDE_RY - 45, text="Side View",
                          font=("Segoe UI", 14, "italic"), fill="#000000")
        canvas.create_oval(ex - SIDE_RX, ey - SIDE_RY, ex + SIDE_RX, ey + SIDE_RY,
                          outline="#333333", width=3)

        # inner arc (the extra curve visible inside the body in the
        # original sketch, like an underside/hull line). Adjust
        # INNER_ARC_* below if the curve's shape/position needs tuning.
        INNER_ARC_RX = SIDE_RX * 0.75
        INNER_ARC_RY = SIDE_RY * 0.7
        inner_cx = ex - SIDE_RX * 0.05
        inner_cy = ey - SIDE_RY * -0.9
        canvas.create_arc(inner_cx - INNER_ARC_RX, inner_cy - INNER_ARC_RY,
                         inner_cx + INNER_ARC_RX, inner_cy + INNER_ARC_RY,
                         start=15, extent=145, style=tk.ARC,
                         outline="#333333", width=2)

        # emergency vent: a small grille icon (3x3 grid) straddling the
        # ellipse boundary at VENT_ANGLE_DEG
        vx, vy = POSITION_COORDS["emergency_vent"]
        vs = 10  # half-size of the vent square
        canvas.create_rectangle(vx - vs, vy - vs, vx + vs, vy + vs,
                               outline="#333333", width=2, fill="white")
        for i in (1, 2):  # 3x3 grille lines
            gx = vx - vs + i * (2 * vs) / 3
            canvas.create_line(gx, vy - vs, gx, vy + vs, fill="#333333")
            gy = vy - vs + i * (2 * vs) / 3
            canvas.create_line(vx - vs, gy, vx + vs, gy, fill="#333333")

        # safety tether: a rectangle straddling the ellipse boundary at
        # TETHER_ANGLE_DEG, oriented along that same radial direction
        tx3, ty3 = POSITION_COORDS["safety_tether"]
        t_rad = math.radians(TETHER_ANGLE_DEG)
        t_dx, t_dy = math.cos(t_rad), math.sin(t_rad)   # outward direction
        t_perp_dx, t_perp_dy = -t_dy, t_dx
        t_len, t_wid = 10, 10 # Size of the safety tether rectangle (half-length, half-width)
        t_corners = [
            tx3 + t_dx * t_len + t_perp_dx * t_wid, ty3 + t_dy * t_len + t_perp_dy * t_wid,
            tx3 - t_dx * t_len + t_perp_dx * t_wid, ty3 - t_dy * t_len + t_perp_dy * t_wid,
            tx3 - t_dx * t_len - t_perp_dx * t_wid, ty3 - t_dy * t_len - t_perp_dy * t_wid,
            tx3 + t_dx * t_len - t_perp_dx * t_wid, ty3 + t_dy * t_len - t_perp_dy * t_wid,
        ]
        canvas.create_polygon(t_corners, outline="#333333", width=2, fill="#f0f0f0")

    build_diagram_canvas(diagram_panel)

    tk.Label(content_frame, text="Settings (raw values)", font=("Segoe UI", 14, "bold")).pack(pady=(14, 4))

    fields_frame = tk.Frame(content_frame)
    fields_frame.pack(padx=10)
    # Two side-by-side groups of rows (label+value), each its own pair
    # of grid columns - this is what lets the settings wrap into a
    # second column instead of stacking into one long list.
    WRAP_AFTER = 6
    LEFT_LABEL_WIDTH = 24    # column-pair 0 (left side)
    RIGHT_LABEL_WIDTH = 21   # column-pair 1 (right side) - smaller, so
                             # the box sits closer to the text there
    COLUMN_GAP = 24  # extra space BEFORE the right column starts - the
                     # number to change to push the two columns apart
    fields_frame.columnconfigure(1, minsize=130)
    fields_frame.columnconfigure(3, minsize=130)

    def grid_slot(index):
        """Turns a plain 0,1,2,3... counter into (row, label_col,
        value_col, label_width, label_padx) - after WRAP_AFTER rows,
        it starts a new column pair instead of continuing downward."""
        row = index % WRAP_AFTER
        pair = index // WRAP_AFTER
        label_width = LEFT_LABEL_WIDTH if pair == 0 else RIGHT_LABEL_WIDTH
        label_padx = (4, 4) if pair == 0 else (COLUMN_GAP, 4)
        return row, pair * 2, pair * 2 + 1, label_width, label_padx

    # (key, label, register) - plain number entry fields
    FIELD_INFO = [
        ("pos_mid", "Pos Mid (-16384..16384)", 0xC2),
        ("pos_max", "Pos Max (-16384..16384)", 0xB0),
        ("pos_min", "Pos Min (-16384..16384)", 0xB2),
        ("speed", "Speed (0-4095)", 0x54),
        ("speed_up", "Time Spd Up (ms)", 0xDC),
        ("speed_dn", "Time Spd Dn (ms)", 0xDE),
        ("stream_time", "Stream Time (0-12000)", 0x2E),
    ]

    SAMPLE_POINT_REGISTER = 0x40
    SAMPLE_POINT_LABELS = {0: "50", 1: "87.5"}
    SAMPLE_POINT_VALUES = {label: raw for raw, label in SAMPLE_POINT_LABELS.items()}

    BAUD_RATE_REGISTER = 0x38
    BAUD_RATE_LABELS = {
        0: "1000", 1: "800", 2: "750", 3: "500",
        4: "400", 5: "250", 6: "200", 7: "150", 8: "125",
    }
    BAUD_RATE_VALUES = {label: raw for raw, label in BAUD_RATE_LABELS.items()}

    STREAM_MODE_REGISTER = 0x30
    STREAM_MODE_LABELS = {0: "Off", 1: "On"}
    STREAM_MODE_VALUES = {label: raw for raw, label in STREAM_MODE_LABELS.items()}

    # IDs: written with everything else, but only take effect after a
    # power cycle (per the manual), so their true confirmation is the
    # display at the top after replug. DroneCAN ranges.
    ID1_REGISTER, ID2_REGISTER = 0x32, 0x3E

    entries = {}
    next_index = 0  # shared counter across ALL rows below

    # --- ID rows (entry fields, validated 1-127 / 0-127 on save) ---
    # Moved to run FIRST so ID1/ID2 are the first two rows shown.
    id_entries = {}
    ID_WRITE_INFO = [
        ("ID1 (Actuator ID)", ID1_REGISTER, 1, 127),
        ("ID2 (CAN Node ID)", ID2_REGISTER, 0, 127),
    ]
    for label, register, lo_limit, hi_limit in ID_WRITE_INFO:
        r, lc, vc, lw, lpad = grid_slot(next_index); next_index += 1
        tk.Label(fields_frame, text=f"{label} ({lo_limit}-{hi_limit})",
                 width=lw, font=("Segoe UI", 11), anchor="w").grid(row=r, column=lc, sticky="w", padx=lpad, pady=3)
        entry = tk.Entry(fields_frame, width=11, font=("Segoe UI", 11))
        entry.insert(0, "-")
        entry.grid(row=r, column=vc, padx=4, pady=3, sticky="ew")
        id_entries[label] = entry

    # --- plain entry rows ---
    for key, label, register in FIELD_INFO:
        row, label_col, value_col, label_width, label_padx = grid_slot(next_index)
        next_index += 1
        tk.Label(fields_frame, text=label, width=label_width, font=("Segoe UI", 11), anchor="w").grid(
            row=row, column=label_col, sticky="w", padx=label_padx, pady=3)
        entry = tk.Entry(fields_frame, width=11, font=("Segoe UI", 11))
        entry.insert(0, "-")
        entry.grid(row=row, column=value_col, padx=4, pady=3, sticky="ew")
        entries[key] = (entry, register)

    # --- dropdown rows ---

    sm_row, sm_lc, sm_vc, sm_lw, sm_lpad = grid_slot(next_index); next_index += 1
    tk.Label(fields_frame, text="Stream Mode", width=sm_lw, font=("Segoe UI", 11), anchor="w").grid(
        row=sm_row, column=sm_lc, sticky="w", padx=sm_lpad, pady=3)
    stream_mode_var = tk.StringVar(value="-")
    stream_mode_menu = tk.OptionMenu(fields_frame, stream_mode_var, *STREAM_MODE_LABELS.values())
    stream_mode_menu.config(width=7, font=("Segoe UI", 11))
    stream_mode_menu.grid(row=sm_row, column=sm_vc, padx=4, pady=3, sticky="ew")
    stream_mode_menu["menu"].config(font=("Segoe UI", 11))

    sp_row, sp_lc, sp_vc, sp_lw, sp_lpad = grid_slot(next_index); next_index += 1
    tk.Label(fields_frame, text="Sample Point [%]", width=sp_lw, font=("Segoe UI", 11), anchor="w").grid(
        row=sp_row, column=sp_lc, sticky="w", padx=sp_lpad, pady=3)
    sample_point_var = tk.StringVar(value="-")
    sample_point_menu = tk.OptionMenu(fields_frame, sample_point_var, *SAMPLE_POINT_LABELS.values())
    sample_point_menu.config(width=7, font=("Segoe UI", 11))
    sample_point_menu.grid(row=sp_row, column=sp_vc, padx=4, pady=3, sticky="ew")
    sample_point_menu["menu"].config(font=("Segoe UI", 11))

    br_row, br_lc, br_vc, br_lw, br_lpad = grid_slot(next_index); next_index += 1
    tk.Label(fields_frame, text="Baud Rate [kbps]", width=br_lw, font=("Segoe UI", 11), anchor="w").grid(
        row=br_row, column=br_lc, sticky="w", padx=br_lpad, pady=3)
    baud_rate_var = tk.StringVar(value="-")
    baud_rate_menu = tk.OptionMenu(fields_frame, baud_rate_var, *BAUD_RATE_LABELS.values())
    baud_rate_menu.config(width=7, font=("Segoe UI", 11))
    baud_rate_menu.grid(row=br_row, column=br_vc, padx=4, pady=3, sticky="ew")
    baud_rate_menu["menu"].config(font=("Segoe UI",11))

    # --- the single Save button ---

    def gather_and_validate():
        """Reads every field, validates it, and returns two views of the
        same data: `to_write` (register -> value, for writing to the
        servo) and `settings` (profile-key -> value, for saving to the
        JSON profile). Returns (to_write, settings, error_message).
        If error_message is not None, nothing was written anywhere -
        both callers must check this before doing anything."""
        to_write = {}   # register -> value
        settings = {}   # profile key -> value

        for key, (entry, register) in entries.items():
            raw = entry.get().strip()
            try:
                value = int(raw)
            except ValueError:
                return None, None, f"'{raw}' for {key} isn't a whole number. Nothing saved."
            to_write[register] = value
            settings[key] = value

        sp_label = sample_point_var.get()
        if sp_label not in SAMPLE_POINT_VALUES:
            return None, None, "Pick a Sample Point first. Nothing saved."
        to_write[SAMPLE_POINT_REGISTER] = SAMPLE_POINT_VALUES[sp_label]
        settings["sample_point"] = SAMPLE_POINT_VALUES[sp_label]

        br_label = baud_rate_var.get()
        if br_label not in BAUD_RATE_VALUES:
            return None, None, "Pick a Baud Rate first. Nothing saved."
        to_write[BAUD_RATE_REGISTER] = BAUD_RATE_VALUES[br_label]
        settings["baud_rate"] = BAUD_RATE_VALUES[br_label]

        sm_label = stream_mode_var.get()
        if sm_label not in STREAM_MODE_VALUES:
            return None, None, "Pick a Stream Mode first. Nothing saved."
        to_write[STREAM_MODE_REGISTER] = STREAM_MODE_VALUES[sm_label]
        settings["stream_mode"] = STREAM_MODE_VALUES[sm_label]

        for label, register, lo_limit, hi_limit in ID_WRITE_INFO:
            raw = id_entries[label].get().strip()
            try:
                value = int(raw)
            except ValueError:
                return None, None, f"'{raw}' for {label} isn't a whole number. Nothing saved."
            if not (lo_limit <= value <= hi_limit):
                return None, None, f"{label} must be {lo_limit}-{hi_limit}. Nothing saved."
            to_write[register] = value
            settings["id1" if label.startswith("ID1") else "id2"] = value

        return to_write, settings, None

    def on_save_to_servo():
        """Writes the current field values to the real servo and
        confirms them. Does NOT touch the saved profile file."""
        to_write, _settings, error = gather_and_validate()
        if error:
            status_var.set(error)
            return

        status_var.set(f"Writing {len(to_write)} settings to servo...")
        root.update()

        confirmed, error = hitec.write_all(to_write)
        if error:
            status_var.set(error)
            return

        # Update the UI from what the servo actually reports back.
        failed = []
        for key, (entry, register) in entries.items():
            back = confirmed.get(register)
            set_entry(entry, back, placeholder="??")
            if back is None:
                failed.append(key)

        sp_back = confirmed.get(SAMPLE_POINT_REGISTER)
        sample_point_var.set(SAMPLE_POINT_LABELS.get(sp_back, "??" if sp_back is None else str(sp_back)))
        if sp_back is None:
            failed.append("sample_point")

        br_back = confirmed.get(BAUD_RATE_REGISTER)
        baud_rate_var.set(BAUD_RATE_LABELS.get(br_back, "??" if br_back is None else str(br_back)))
        if br_back is None:
            failed.append("baud_rate")

        sm_back = confirmed.get(STREAM_MODE_REGISTER)
        stream_mode_var.set(STREAM_MODE_LABELS.get(sm_back, "??" if sm_back is None else str(sm_back)))
        if sm_back is None:
            failed.append("stream_mode")

        if failed:
            status_var.set(f"Written to servo, but couldn't confirm: {', '.join(failed)}. "
                           "IDs need an unplug/replug to apply.")
        else:
            status_var.set("Written to servo and confirmed. "
                           "Press Read ID button to see updated ID's")

    def on_save_to_profile():
        """Saves the current field values into the selected profile's
        JSON entry. Does NOT touch the servo at all - no serial
        communication happens here."""
        profile_name = profile_var.get()
        if not profile_name or profile_name == "-" or profile_name not in profiles:
            status_var.set("Select or add a profile first. Nothing saved.")
            return

        _to_write, settings, error = gather_and_validate()
        if error:
            status_var.set(error)
            return

        profiles[profile_name] = settings
        save_profiles(profiles)
        status_var.set(f"Saved to profile '{profile_name}'.")

    tk.Button(content_frame, text="Save to servo only", command=on_save_to_servo,
             font=("Segoe UI", 13), width=20).pack(pady=(10, 4))

    tk.Label(content_frame, textvariable=status_var, fg="gray", font=("Segoe UI", 15),
             wraplength=380, justify="center").pack(pady=(8, 8))

    state = {"connected_port": None}

    def fill_settings_fields(settings):
        """Populate the settings fields from a plain dict of
        key -> raw value - used for loading a SAVED profile. Does not
        touch the ID display at the top (that's live-read separately)."""
        for key, (entry, _register) in entries.items():
            val = settings.get(key)
            set_entry(entry, val)

        sp_val = settings.get("sample_point")
        sample_point_var.set(SAMPLE_POINT_LABELS.get(sp_val, "-" if sp_val is None else str(sp_val)))

        br_val = settings.get("baud_rate")
        baud_rate_var.set(BAUD_RATE_LABELS.get(br_val, "-" if br_val is None else str(br_val)))

        sm_val = settings.get("stream_mode")
        stream_mode_var.set(STREAM_MODE_LABELS.get(sm_val, "-" if sm_val is None else str(sm_val)))

        for label, source_key in (("ID1 (Actuator ID)", "id1"), ("ID2 (CAN Node ID)", "id2")):
            entry = id_entries[label]
            val = settings.get(source_key)
            set_entry(entry, val)

    def on_profile_selected(_event=None):
        """Bound to the profile dropdown's <<ComboboxSelected>> event.
        Selecting a profile loads its SAVED settings into the fields.
        No serial communication happens here - the servo isn't touched
        until you click Save."""
        name = profile_var.get()
        if name and name != "-" and name in profiles:
            fill_settings_fields(profiles[name])
            status_var.set(f"'{name}' profile loaded. Edit as needed, then Save "
                           "to write it to the servo.")
        update_diagram_marker()

    def on_add_profile():
        """Handler for the 'Add' button: creates a new profile with
        DEFAULT_PROFILE_SETTINGS if the typed name doesn't already
        exist, saves profiles.json, refreshes the dropdown's list of
        values, selects the new profile, and loads its fields (via
        on_profile_selected). If the name already exists, just selects
        and loads the existing one instead of overwriting it."""
        name = new_profile_entry.get().strip()
        if not name:
            status_var.set("Type a profile name first.")
            return
        if name not in profiles:
            profiles[name] = dict(DEFAULT_PROFILE_SETTINGS)
            save_profiles(profiles)
            profile_box.config(values=list(profiles.keys()))
        profile_var.set(name)
        new_profile_entry.delete(0, tk.END)
        on_profile_selected()

    def on_delete_profile():
        """Handler for the 'Delete' button.
        Deletes the currently selected profile from the saved file.
        Does not touch the servo."""
        name = profile_var.get()
        if not name or name == "-" or name not in profiles:
            status_var.set("Select a profile to delete first.")
            return

        del profiles[name]
        save_profiles(profiles)
        profile_box.config(values=list(profiles.keys()))
        profile_var.set("-")

        for entry, _register in entries.values():
            set_entry(entry, None)
        sample_point_var.set("-")
        baud_rate_var.set("-")
        stream_mode_var.set("-")
        for entry in id_entries.values():
            set_entry(entry, None)

        update_diagram_marker()
        status_var.set(f"Deleted profile '{name}'.")

    profile_box.bind("<<ComboboxSelected>>", on_profile_selected)

    def poll():
        """Runs every second (via root.after) to detect the Hitec servo
        being plugged in or unplugged, purely by USB presence - it never
        opens the serial port or talks to the servo. Clears all fields
        and shows 'Waiting for servo...' on disconnect; on a fresh
        connection, resets the profile selector to '-' and prompts the
        user to pick which profile this servo is, since the app can't
        tell which physical actuator just appeared."""
        port = hitec.find_port()

        if port is None:
            if state["connected_port"] is not None:
                state["connected_port"] = None
                for entry, _r in entries.values():
                    set_entry(entry, None)
                sample_point_var.set("-")
                baud_rate_var.set("-")
                stream_mode_var.set("-")
                for entry in id_entries.values():
                    set_entry(entry, None)
                profile_var.set("-")
                update_diagram_marker()
                status_var.set("Waiting for servo...")

        elif port != state["connected_port"]:
            # A new servo just appeared. This is just a USB presence
            # check (no serial communication at all) - select which
            # profile this is, and the fields load from the saved
            # profile, not a live read.
            state["connected_port"] = port
            profile_var.set("-")
            update_diagram_marker()
            status_var.set(f"Servo detected on {port}. Select which profile this is.")

        root.after(1000, poll)

    def poll_global_status():
        """Runs every second (via root.after) to update the top-of-window
        status bar with which servo(s) are currently connected - Hitec,
        CubeMars, both, or neither - based on a simple USB presence
        check for each adapter."""
        hitec_port = hitec.find_port()
        cubemars_port = cubemars.find_port()

        if hitec_port and cubemars_port:
            global_status_var.set(f"Hitec on {hitec_port}, CubeMars on {cubemars_port} — both connected.")
        elif hitec_port:
            global_status_var.set(f"Hitec servo connected on {hitec_port}.")
        elif cubemars_port:
            global_status_var.set(f"CubeMars servo connected on {cubemars_port}.")
        else:
            global_status_var.set("No servo connected.")

        root.after(1000, poll_global_status)

    build_cubemars_tab(cubemars_tab, cubemars)
    poll_global_status()
    poll()
    root.mainloop()

if __name__ == "__main__":
    main()