import krpc
import time
from simple_pid import PID
conn = krpc.connect(name='Sub-orbital flight')
ui = conn.ui
vessel = conn.space_center.active_vessel
refframe = vessel.orbit.body.reference_frame
control = conn.space_center.Control
ap = vessel.auto_pilot
canvas = conn.ui.stock_canvas


apoapsis = 85000  # Flight altitude || CANNOT BE LESS THAN 6000 or the autopilot might not work as intended
start_sb_calculation = apoapsis-1000  # Altitude to start calculating the suicide burn, you usually don't need to change this
countdown = False  # Whether to do the countdown or not

pid_descent = PID(0.05, 0.01, 0.015, setpoint=0) # Initiate PID parameters, you can tweak those if you want to find a better control
pid_landing = PID(0.05, 0.01, 0.015, setpoint=0)
pid_landing.proportional_on_measurement = True

# Get the size of the game window in pixels
screen_size = canvas.rect_transform.size
# Add a panel to contain the UI elements
panel = canvas.add_panel()
# Position the panel on the left of the screen
rect = panel.rect_transform
rect.size = (200, 100)
rect.position = (110-(screen_size[0]/2), 0)

# Add a button to set the throttle to maximum
launch_button = panel.add_button("Launch")
launch_button.rect_transform.position = (0, 20)

# Add some text displaying the total engine thrust
text = panel.add_text("State: ")
text.rect_transform.position = (0, -20)
text.color = (1, 1, 1)
text.size = 18

# Set up a stream to monitor the throttle button
launch_button_clicked = conn.add_stream(getattr, launch_button, 'clicked')

vertical_speed = conn.add_stream(getattr, vessel.flight(refframe), 'vertical_speed')
surface_altitude = conn.add_stream(getattr, vessel.flight(), 'surface_altitude')
throttle = conn.add_stream(getattr, vessel.control, "throttle")
vessel_available_thrust = conn.add_stream(getattr, vessel, "available_thrust")
vessel_thrust_ = conn.add_stream(getattr, vessel, "thrust")


# method used to display messages both to Kerbal UI and Python console
def message(text, duration=3):
    text = str(text)
    ui.message(text, duration=duration)
    print(text)


# method used by PID to control the throttle value during the landing burn
def update_throttle(p):
    if vertical_speed() > -2:
        vessel.control.throttle = 0.0
    elif vertical_speed() < -5:
        vessel.control.throttle = 1.0
    else:
        vessel.control.throttle += p
    text.content = str(p)
    vs = vertical_speed()
    vs += 3 # means that PID will try to keep the vs at -3
    return vs

# method used to control the throttle value during the suicide burn
def control_descent(p):
    throttle_value = vessel.control.throttle + p
    if vertical_speed() > -3:
        vessel.control.throttle = 0.0
    else:
        vessel.control.throttle += p
    _min_height = min_height_for_burn()
    diff = surface_altitude() - _min_height
    # diff -= 40 # means that PID will try to keep the difference within 40 meters
    if diff > 5:
        vessel.control.throttle = 0
    elif diff < -5:
        vessel.control.throttle = 1
    return diff

# returns the altitude to perform the suicide burn
def min_height_for_burn():
    g = 9.81  # Gravitation force at the vessel
    vessel_thrust = vessel_thrust_() # Vessel current thrust
    engine_thrust = vessel_available_thrust()  # Available engine thrust
    if vessel_thrust > 0:
        a = (engine_thrust / vessel_thrust) - g  # net acceleration
    else:
        a = 0 - g
    v_0 = vertical_speed()  # initial velocity

    __min_height = abs((v_0 / 2) * (v_0 / a))
    text.content = "D: {0:.1f} - T: {1:.2f}".format(surface_altitude() - __min_height, throttle())
    # print('Min Height:', __min_height)
    # print('Current Alt:', surface_altitude(), '\n')
    return __min_height

def launch():
    message('Initiating launch')
    message('Target Pitch and Heading Engaged')
    vessel.auto_pilot.target_pitch_and_heading(90, 245)
    set_engine_gimbal_limit(0.5)
    ap.target_roll = 0
    vessel.control.gear = False # just to guarantee that the landing gear is up

    countdown_timer = 10
    if countdown is True:
        message('Initiating countdown')
        text.content = "State: Countdown"
        time.sleep(2)
        for i in range(10):
            message("{0}".format(countdown_timer))
            countdown_timer -= 1
            time.sleep(1)
            if countdown_timer == 3:
                message('Engine Start')
                vessel.control.activate_next_stage()
                vessel.auto_pilot.engage()
                vessel.control.throttle = 1
    else:
        message('Engine Start')
        vessel.control.activate_next_stage()
        vessel.auto_pilot.engage()
        vessel.control.throttle = 1

    message('Launch', 10)
    text.content = "State: Launch"
    vessel.control.activate_next_stage()
    monitor_ascent_turns()


def monitor_ascent_turns():
    if apoapsis > 1000:
        while surface_altitude() < 1000:
            pass

    vessel.auto_pilot.target_pitch_and_heading(75, 245)
    message('Turning')

    if apoapsis > 5000:
        while surface_altitude() < 5000:
            pass

    vessel.auto_pilot.target_pitch_and_heading(65, 245)
    message('Turning')
    wait_for_apoapsis()

def wait_for_apoapsis():
    # waits for the apoapsis altitude to be reached
    apoapsis_altitude = conn.get_call(getattr, vessel.orbit, 'apoapsis_altitude')
    expr = conn.krpc.Expression.greater_than(
        conn.krpc.Expression.call(apoapsis_altitude),
        conn.krpc.Expression.constant_double(apoapsis))
    event = conn.krpc.add_event(expr)
    with event.condition:
        event.wait()

    message('MECO')
    text.content = "State: Coasting to Apoasis"

    # points the vessel to retrograde
    vessel.control.throttle = 0
    ap.disengage()
    ap.sas = True
    time.sleep(1)
    ap.sas_mode = ap.sas_mode.prograde

    # wait until apoapsis is reached
    if surface_altitude() < apoapsis:
        while vertical_speed() > 0:
            pass

    message('Reached Apoapsis altitude')
    text.content = "State: Waiting for Suicide Burn"
    set_rcs(True)
    ap.disengage()
    ap.sas = True
    time.sleep(1)
    ap.sas_mode = ap.sas_mode.retrograde
    print(ap.sas_mode)
    message("Autopilot Disengaged. Pointing to Retrograde")
    vessel.control.throttle = 0.1
    time.sleep(5)
    vessel.control.throttle = 0.0
    vessel.control.activate_next_stage() # decouple fairings
    wait_for_sb_calculation()

def wait_for_sb_calculation():
    ap.engage()
    ap.target_roll = 0
    time.sleep(5)
    ap.disengage()
    ap.sas = True
    time.sleep(1)
    ap.sas_mode = ap.sas_mode.retrograde
    # wait until the start_sb_calculation altitude is reached, then, begin suicide burn calculation
    # srf_altitude = conn.get_call(getattr, vessel.flight(), 'surface_altitude')
    # expr = conn.krpc.Expression.less_than(
    #     conn.krpc.Expression.call(srf_altitude),
    #     conn.krpc.Expression.constant_double(start_sb_calculation))
    # event = conn.krpc.add_event(expr)
    # with event.condition:
    #     event.wait()

    min_height = 0

    message("Expecting Suicide Burn to Begin at: {0:.2f}m AGL".format((surface_altitude() - min_height_for_burn())/2))

    while min_height < surface_altitude():
        min_height = min_height_for_burn()
    start_suicide_burn()


def start_suicide_burn():
    set_airbrake_control(True)
    message('Starting Suicide Burn')
    text.content = "State: Executing Suicide Burn"
    vessel.control.throttle = 1.0

    # vessel.control.set_action_group(1, True)

    v = control_descent(1)
    while surface_altitude() > 5 and vertical_speed() < -1:
        # if vessel.control.rcs is True and surface_altitude() < 15000:
        #     set_rcs(False)
        if vessel.control.gear is False and surface_altitude() < 300:
            vessel.control.gear = True
            message('Deploying landing gear')
        if ap.sas_mode == ap.sas_mode.retrograde and surface_altitude() < 300:
            ap.sas_mode = ap.sas_mode.stability_assist
            ap.sas = False
            ap.engage()
            vessel.auto_pilot.target_pitch_and_heading(90, 245)

        # assume we have a system we want to control in update_throttle
        control = pid_descent(v)
        # feed the PID output to the system and get its current value
        v = control_descent(control)
    switch_to_touchdown_control()


def switch_to_touchdown_control():
    lock_engine_gimbal(False)
    set_airbrake_control(False)
    message('Transitioned to Touchdown Control')
    text.content = "State: Touchdown Control"
    vessel.control.throttle = 1.0
    while vertical_speed() < -6:
        pass

    set_engine_thrust_limit(0.5)
    # assume we have a system we want to control in update_throttle
    v = update_throttle(0)
    while surface_altitude() > 8 or vertical_speed() < -0.5:
        # compute new ouput from the PID according to the systems current value
        control = pid_landing(v)

        # feed the PID output to the system and get its current value
        v = update_throttle(control)
    switch_to_landed_mode()


def switch_to_landed_mode():
    message('Landed')
    message("Engines Off.")
    text.content = "State: Landed"
    vessel.control.set_action_group(1, False)
    vessel.control.throttle = 0
    time.sleep(3)
    set_rcs(False)
    quit()

def set_airbrake_control(condition):
    airbrakes = vessel.parts.with_name('airbrake1')
    for i in range(len(airbrakes)):
        airbrakes[i].control_surface.yaw_enabled = condition
        airbrakes[i].control_surface.pitch_enabled = condition

def set_rcs(condition):
    vessel.control.rcs = condition

def lock_engine_gimbal(condition):
    engine = vessel.parts.with_name('SSME')
    engine[0].engine.gimbal_locked = condition

def set_engine_thrust_limit(limit):
    engine = vessel.parts.with_name('SSME')
    engine[0].engine.thrust_limit = limit

def set_engine_gimbal_limit(limit):
    engine = vessel.parts.with_name('SSME')
    engine[0].engine.gimbal_limit = limit

while True:
    if launch_button_clicked():
        # launch()
        wait_for_sb_calculation()
        launch_button.clicked = False