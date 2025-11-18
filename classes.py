# Group 3

from ophyd import Device, EpicsSignal, EpicsSignalRO
from ophyd import Component as Cpt

from functools import partial
import time

import bluesky.plan_stubs as bps
from bluesky.plans import scan

class PowerSupply(Device):
    cmdPwrOn = Cpt(EpicsSignal, "cmdPwrOn", kind = "config")   # accensione
    cmdPwrOff = Cpt(EpicsSignal, "cmdPwrOff", kind = "config")   # spegnimento
    wrCurNoRamp = Cpt(EpicsSignal, "wrCurNoRamp", kind = "hinted")   # imposta corrente senza rampa
    wrCurRamp = Cpt(EpicsSignal, "wrCurRamp", kind = "hinted")   # imposta corrente con rampa HW
    wrSlewRate = Cpt(EpicsSignal, "wrSlewRate", kind = "config")   # imposta slew rate (A/s)

    rdCur = Cpt(EpicsSignalRO, "rdCur", kind = "hinted")   # corrente letta
    rdCurSlewRate = Cpt(EpicsSignalRO, "rdCurSlewRate", kind = "config")   # slew rate letto
    rdHeatSinkTmp = Cpt(EpicsSignalRO, "rdHeatSinkTmp", kind = "normal")   # temp heatsink
    rdHeatShuntTmp = Cpt(EpicsSignalRO, "rdHeatShuntTmp", kind = "normal")   # temp shunt
    rdVolt = Cpt(EpicsSignalRO, "rdVolt", kind = "normal")   # tensione

    botton = Cpt(EpicsSignalRO, "botton", kind = "config")
    done = Cpt(EpicsSignalRO, "done", kind = "hinted")
    boundary = Cpt(EpicsSignalRO, "boundary", kind = "config")

    def turn_on(self):
        """
        Turn ON
        """
        self.cmdPwrOn.put(1)
        
    def turn_off(self):
        """
        Turn OFF
        """
        self.cmdPwrOff.put(1)

    def set_current_now(self, amps):
        """
        Set current without ramp
        """
        self.wrCurNoRamp.put(amps)

    def set_current_ramp(self, amps):
        """
        Set current with hardware ramp
        """
        self.wrCurRamp.put(amps)

    def set_slew(self, a_per_s):          # A/s
        self.wrSlewRate.put(a_per_s)

    def read_current(self):
        """
        Read current
        """
        return float(self.rdCur.get())
        
    def read_slew(self):
        """
        Read slew
        """
        return float(self.rdCurSlewRate.get())
        
    def read_temps(self):
        """
        Measure temperature readings
        """
        return (float(self.rdHeatSinkTmp.get()), float(self.rdHeatShuntTmp.get()))
        
    def read_voltage(self):
        """
        Read voltage
        """
        return float(self.rdVolt.get())
        
    def read_status(self):
        return {
            "I_set_no_ramp":   float(self.wrCurNoRamp.get()),
            "I_set_ramp":      float(self.wrCurRamp.get()),
            "I_read":          float(self.rdCur.get()),
            "slew_rate_set":   float(self.wrSlewRate.get()),
            "slew_rate_read":  float(self.rdCurSlewRate.get()),
            "T_heatsink":      float(self.rdHeatSinkTmp.get()),
            "T_shunt":         float(self.rdHeatShuntTmp.get()),
            "V":               float(self.rdVolt.get()),
            "done":            int(self.done.get()),
            "botton":          int(self.botton.get()),
            "boundary":        int(self.boundary.get()),
        }

def ramp_scan(ps, start, stop, step, pre_wait_s=3, hold_s=2, tol=0.02, base_timeout=5):
    npts = int(abs((stop-start)/step)) + 1
    yield from bps.mv(ps.cmdPwrOn, 1)

    custom = partial(
        _per_step,
        ps=ps,
        pre_wait_s=pre_wait_s,
        hold_s=hold_s,
        tol=tol,
        base_timeout=base_timeout
    )

    yield from scan([ps], ps.wrCurRamp, start, stop, npts, per_step=custom)


def await_in_band(ps, target, tol=0.02, dwell=0.1, stable_time=0.4, max_time=10):
    # attende che rdCur sia entro ±tol dal target per un po' di tempo
    need = max(1, int(stable_time/dwell))
    ok = 0; t0 = time.time()
    while True:
        err = abs(float(ps.rdCur.get()) - float(target))
        ok = ok+1 if err <= tol else 0
        if ok >= need: return
        if time.time() - t0 > max_time:
            raise TimeoutError(f"rdCur non entro ±{tol} A da {target}")
        yield from bps.sleep(dwell)

def _per_step(detectors, motor, step, *, ps, pre_wait_s, hold_s, tol, base_timeout):
    yield from bps.sleep(pre_wait_s)
    yield from bps.mv(motor, step)

    try:
        slew = float(ps.rdCurSlewRate.get())
        if slew <= 0:
            slew = 0.1
    except Exception:
        slew = 0.1

    last = float(ps.rdCur.get())
    dyn_to = base_timeout + abs(step - last)/slew + 2.0

    yield from await_in_band(ps, step, tol=tol, max_time=dyn_to)

    yield from bps.sleep(hold_s)
    return (yield from bps.trigger_and_read(list(detectors) + [motor]))

class Gaussmeter(Device):
   # READ
   bfield = Cpt(EpicsSignalRO, "rdBfield")
   volt = Cpt(EpicsSignalRO, "rdVolt")
   idn = Cpt(EpicsSignalRO, "rdIdn", kind = "config")
   status = Cpt(EpicsSignalRO, "rdStatus", kind = "config")

   # SWITCH
   onctrl = Cpt(EpicsSignal, "wrOnctrl", kind = "config")

   # COMMANDS 
   def switch(self, bi):
      """
      1: On
      0: Off
      """
      return self.onctrl.put(bi)

   def read_mag_field(self):
       """
       Returns magnetic field measurement
       """
       return self.bfield.get()
   
   def all(self):
       """
       Returns a directory
       """
       return {
         "mag_field": self.bfield.get(),
         "voltage" : self.volt.get(),
         "onctrl" : self.onctrl.get(),
         "idn" : self.idn.get(),
         "status" : self.status.get()
       }