"""Checks shared by the flight scripts: battery, temperature, countdown, landing."""

import time

MIN_BATTERY = 25      # percent; the drone itself refuses takeoff below ~10%
MAX_TEMP_C = 90       # refuse takeoff if the drone is this hot (it heats up idling on the ground)


def preflight(tello, ignore_temp: bool = False, min_battery: int = MIN_BATTERY,
              max_temp: int = MAX_TEMP_C):
    """Return None if it's OK to take off, else the reason not to."""
    battery = tello.get_battery()
    if battery < min_battery:
        return f"Battery {battery}% is below {min_battery}%, charge before flying."
    temp = tello.get_highest_temperature()
    if temp >= max_temp and ignore_temp:
        print(f"WARNING: drone is at {temp}C (limit {max_temp}C), flying anyway (--ignore-temp).")
    elif temp >= max_temp:
        return f"Drone is at {temp}C (limit {max_temp}C). Power it off to cool before flying."
    return None


def countdown(seconds: int, sleep=time.sleep) -> bool:
    """Count down to takeoff. False if Ctrl+C cancelled it."""
    try:
        for remaining in range(seconds, 0, -1):
            print(f"Taking off in {remaining}... (Ctrl+C to cancel)")
            sleep(1)
    except KeyboardInterrupt:
        print("\nCancelled before takeoff. Nothing flew.")
        return False
    return True


def land_now(tello) -> None:
    """Best-effort landing, used from `finally` blocks."""
    try:
        tello.land()
    except Exception as exc:
        print(f"Land command failed: {exc!r}. Grab the drone or use the "
              f"power button; 'tello.emergency()' cuts motors instantly.")
