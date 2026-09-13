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


POSITION_COORDS = {}

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
    inner = tk.Frame(parent, width=1100, height=760)
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
    tk.Label(inner, textvariable=baud_var, font=("Segoe UI", 14)).pack(pady=(1, 10))

    # Two side-by-side columns: App Config on the left, Motor Config on the right
    columns_frame = tk.Frame(inner)
    columns_frame.pack()

    app_col = tk.Frame(columns_frame)
    app_col.grid(row=0, column=0, sticky="n", padx=(0, 70))

    motor_col = tk.Frame(columns_frame)
    motor_col.grid(row=0, column=1, sticky="n")

    # ---------------- App Config (left column) ----------------

    tk.Label(app_col, text="App Config", font=("Segoe UI", 14, "bold")).pack(pady=(0, 6))

    app_config_grid = tk.Frame(app_col)
    app_config_grid.pack(pady=(10, 14))

    tk.Label(app_config_grid, text="New Controller ID (0-255):", font=("Segoe UI", 12)).grid(
        row=0, column=0, sticky="w", padx=(0, 20), pady=(0, 2))
    new_id_entry = tk.Entry(app_config_grid, width=10, font=("Segoe UI", 12))
    new_id_entry.grid(row=1, column=0, sticky="n", padx=(0, 20), pady=(0, 14))

    tk.Label(app_config_grid, text="New CAN Baud Rate:", font=("Segoe UI", 12)).grid(
        row=0, column=1, sticky="n", pady=(0, 2))
    new_baud_var = tk.StringVar(value="-")
    baud_box = ttk.Combobox(app_config_grid, textvariable=new_baud_var, state="readonly",
                            values=["-"] + list(servo_classes.CubeMarsServo.CUBEMARS_CAN_BAUD_LABELS.values()), width=10, font=("Segoe UI", 12))
    baud_box.grid(row=1, column=1, sticky="n", pady=(0, 14))

    # Motor Timeout and Brake Current stay as plain typed entries, same
    # pattern as everything else. Status Feedback Enable/Rate are handled
    # separately below since Rate should only be editable while Enable is checked.
    appconf_extra_entries = {}

    tk.Label(app_config_grid, text="Motor Timeout (ms)", font=("Segoe UI", 12)).grid(
        row=2, column=0, sticky="n", padx=(0, 20))
    timeout_entry = tk.Entry(app_config_grid, width=10, font=("Segoe UI", 12))
    timeout_entry.grid(row=3, column=0, sticky="n", padx=(0, 20))
    appconf_extra_entries["timeout_ms"] = timeout_entry

    tk.Label(app_config_grid, text="Brake Current on Timeout (A)", font=("Segoe UI", 12)).grid(
        row=2, column=1, sticky="w")
    brake_entry = tk.Entry(app_config_grid, width=10, font=("Segoe UI", 12))
    brake_entry.grid(row=3, column=1, sticky="n")
    appconf_extra_entries["brake_current_timeout"] = brake_entry

    # Status Feedback Enable (checkbox) + Status Feedback Rate (only
    # editable while the checkbox is checked, mirroring how the Rate
    # field is greyed out in the official app until "Send status over
    # CAN" is ticked).
    status_enable_var = tk.IntVar(value=0)

    tk.Label(app_col, text="Status Feedback Rate (Hz)", font=("Segoe UI", 12)).pack(pady=(10, 0))
    status_rate_entry = tk.Entry(app_col, width=15, font=("Segoe UI", 12), state="disabled")
    status_rate_entry.pack(pady=(2, 4))

    def on_status_enable_toggle():
        """Runs whenever the Status Feedback Enable checkbox is
        clicked. Enables the Rate entry only while checked; disabling
        it also clears whatever was typed, so a stale rate value can't
        get sent while the field looks greyed-out/inactive."""
        if status_enable_var.get() == 1:
            status_rate_entry.config(state="normal")
        else:
            status_rate_entry.config(state="normal")
            status_rate_entry.delete(0, tk.END)
            status_rate_entry.config(state="disabled")

    status_enable_check = tk.Checkbutton(app_col, text="Status Feedback Enable",
                                          variable=status_enable_var, font=("Segoe UI", 12),
                                          command=on_status_enable_toggle)
    status_enable_check.pack(pady=(0, 14))

    # ---------------- Motor Config (right column) ----------------

    tk.Label(motor_col, text="Motor Config", font=("Segoe UI", 14, "bold")).pack(pady=(0, 6))

    limits_frame = tk.Frame(motor_col)
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
        left_entry = tk.Entry(limits_frame, width=12, font=("Segoe UI", 12))
        left_entry.grid(row=row, column=1, padx=(0, 20), pady=2)
        mcconf_entries[left_field] = left_entry

        tk.Label(limits_frame, text=servo_classes.CubeMarsServo.CUBEMARS_DISPLAY_NAMES[right_field], font=("Segoe UI", 12)).grid(
            row=row, column=2, sticky="w", padx=(0, 4), pady=4)
        right_entry = tk.Entry(limits_frame, width=12, font=("Segoe UI", 12))
        right_entry.grid(row=row, column=3, pady=2)
        mcconf_entries[right_field] = right_entry

    tk.Label(motor_col, text="Speed", font=("Segoe UI", 13, "bold")).pack(pady=(4, 4))
    speed_frame = tk.Frame(motor_col)
    speed_frame.pack(pady=(0, 10))

    for col, field in enumerate(["speed_kp", "speed_ki"]):
        short_label = servo_classes.CubeMarsServo.CUBEMARS_DISPLAY_NAMES[field].replace("Speed ", "")
        tk.Label(speed_frame, text=short_label, font=("Segoe UI", 12)).grid(
            row=0, column=col * 2, sticky="w", padx=(0 if col == 0 else 20, 4))
        entry = tk.Entry(speed_frame, width=12, font=("Segoe UI", 12))
        entry.grid(row=0, column=col * 2 + 1)
        mcconf_entries[field] = entry

    tk.Label(motor_col, text="Position", font=("Segoe UI", 13, "bold")).pack(pady=(4, 4))
    position_frame = tk.Frame(motor_col)
    position_frame.pack(pady=(0, 14))

    for col, field in enumerate(["position_kp", "position_ki", "position_kd"]):
        short_label = servo_classes.CubeMarsServo.CUBEMARS_DISPLAY_NAMES[field].replace("Position ", "")
        tk.Label(position_frame, text=short_label, font=("Segoe UI", 12)).grid(
            row=0, column=col * 2, sticky="w", padx=(0 if col == 0 else 20, 4))
        entry = tk.Entry(position_frame, width=12, font=("Segoe UI", 12))
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

        for field_name, entry in appconf_extra_entries.items():
            offset = servo_classes.CubeMarsServo.CUBEMARS_APPCONF_OFFSETS[field_name]
            if offset is None:
                set_entry(entry, "-")
            else:
                set_entry(entry, servo_classes.CubeMarsServo.cubemars_get_appconf_field(app_result, field_name))

        # Status Feedback Enable/Rate: only populate these if their offsets
        # have actually been confirmed - otherwise leave the checkbox and
        # rate field exactly as they were (unchecked/disabled).
        enable_offset = servo_classes.CubeMarsServo.CUBEMARS_APPCONF_OFFSETS["status_feedback_enable"]
        rate_offset = servo_classes.CubeMarsServo.CUBEMARS_APPCONF_OFFSETS["status_rate_hz"]

        if enable_offset is not None:
            enabled_value = servo_classes.CubeMarsServo.cubemars_get_appconf_field(app_result, "status_feedback_enable")
            status_enable_var.set(1 if enabled_value else 0)
            on_status_enable_toggle()  # syncs the Rate entry's enabled/disabled state to match

        if rate_offset is not None and status_enable_var.get() == 1:
            rate_value = servo_classes.CubeMarsServo.cubemars_get_appconf_field(app_result, "status_rate_hz")
            set_entry(status_rate_entry, rate_value)

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

        appconf_changes = {}

        if new_id is not None:
            appconf_changes["controller_id"] = new_id
        if new_baud is not None:
            appconf_changes["can_baud_rate"] = new_baud

        for field_name, entry in appconf_extra_entries.items():
            text = entry.get().strip()
            if text:
                offset = servo_classes.CubeMarsServo.CUBEMARS_APPCONF_OFFSETS[field_name]
                if offset is None:
                    status_var.set(f"{field_name.replace('_', ' ')} isn't wired up yet - skipped.")
                    continue
                try:
                    appconf_changes[field_name] = int(text) if field_name != "brake_current_timeout" else float(text)
                except ValueError:
                    status_var.set(f"'{text}' is not a valid value for {field_name}.")
                    return

        # Status Feedback Enable always has a definite state (checked or
        # not), so it's included whenever its offset is confirmed -
        # unlike the plain text entries above, there's no "empty" state
        # to skip. Rate is only sent while the checkbox is checked, since
        # the entry is disabled (and cleared) otherwise.
        enable_offset = servo_classes.CubeMarsServo.CUBEMARS_APPCONF_OFFSETS["status_feedback_enable"]
        if enable_offset is not None:
            appconf_changes["status_feedback_enable"] = status_enable_var.get()

            if status_enable_var.get() == 1:
                rate_offset = servo_classes.CubeMarsServo.CUBEMARS_APPCONF_OFFSETS["status_rate_hz"]
                rate_text = status_rate_entry.get().strip()
                if rate_offset is not None and rate_text:
                    try:
                        appconf_changes["status_rate_hz"] = int(rate_text)
                    except ValueError:
                        status_var.set(f"'{rate_text}' is not a valid value for status rate.")
                        return

        if appconf_changes:
            app_result, app_error = cubemars.cubemars_write_appconf(appconf_changes)
            if app_error:
                status_var.set(app_error)
                return
            id_var.set(f"Controller ID: {app_result[5]}")
            baud_var.set(f"CAN Baud: {servo_classes.CubeMarsServo.CUBEMARS_CAN_BAUD_LABELS.get(app_result[17], 'unknown')}")

            if enable_offset is not None:
                enabled_value = servo_classes.CubeMarsServo.cubemars_get_appconf_field(app_result, "status_feedback_enable")
                status_enable_var.set(1 if enabled_value else 0)
                on_status_enable_toggle()
                rate_offset = servo_classes.CubeMarsServo.CUBEMARS_APPCONF_OFFSETS["status_rate_hz"]
                if rate_offset is not None and status_enable_var.get() == 1:
                    rate_value = servo_classes.CubeMarsServo.cubemars_get_appconf_field(app_result, "status_rate_hz")
                    set_entry(status_rate_entry, rate_value)

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
             font=("Segoe UI", 12), width=18).pack(pady=(6, 4), before=columns_frame)
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
            root.geometry("1100x700")
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

    # --- Vehicle diagram (placeholder) ---
    # The original diagram geometry has been removed for privacy. To use
    # your own vehicle image, save a PNG as vehicle_diagram.png in this
    # same folder — it will display automatically. Until then, a
    # placeholder box is shown instead.
    diagram_state = {"canvas": None, "marker": None}

    DIAGRAM_IMAGE_FILE = os.path.join(APP_DIR, "vehicle_diagram.png")

    def update_diagram_marker():
        """Placeholder no-op — position marking depended on the removed
        diagram geometry. Re-implement this once you have your own
        diagram and know where each profile's marker should sit."""
        return

    def build_diagram_canvas(parent):
        """Shows the user's own vehicle_diagram.png if present, otherwise
        a placeholder box explaining how to add one."""
        canvas = tk.Canvas(parent, width=480, height=780, bg="#f0f0f0",
                        highlightthickness=1, highlightbackground="#f0f0f0")
        canvas.pack(pady=(4, 10))
        diagram_state["canvas"] = canvas

        if os.path.exists(DIAGRAM_IMAGE_FILE):
            img = tk.PhotoImage(file=DIAGRAM_IMAGE_FILE)
            diagram_state["image"] = img  # keep a reference so Tkinter doesn't garbage-collect it
            canvas.create_image(240, 390, image=img)
        else:
            canvas.create_rectangle(20, 20, 460, 760, outline="#999999", dash=(4, 2))
            canvas.create_text(
                240, 390,
                text="No vehicle diagram found.\n\nAdd your own image as:\nvehicle_diagram.png\nin this folder.",
                fill="#999999", font=("Segoe UI", 11), justify="center"
            )

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