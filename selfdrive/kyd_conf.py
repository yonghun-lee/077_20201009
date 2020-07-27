import json
import os

class kyd_conf():
  def __init__(self, CP=None):
    self.conf = self.read_config()
    if CP is not None:
      self.init_config(CP)

  def init_config(self, CP):
    write_conf = False

    # only fetch Kp, Ki, Kf sR and sRC from interface.py if it's a PID controlled car
    if CP.lateralTuning.which() == 'pid':
      if self.conf['Kp'] == "-1":
        self.conf['Kp'] = str(round(CP.lateralTuning.pid.kpV[0],3))
        write_conf = True
      if self.conf['Ki'] == "-1":
        self.conf['Ki'] = str(round(CP.lateralTuning.pid.kiV[0],3))
        write_conf = True
      if self.conf['Kf'] == "-1":
        self.conf['Kf'] = str('{:f}'.format(CP.lateralTuning.pid.kf))
        write_conf = True
    
    if self.conf['steerRatio'] == "-1":
      self.conf['steerRatio'] = str(round(CP.steerRatio,3))
      write_conf = True
    
    if self.conf['steerRateCost'] == "-1":
      self.conf['steerRateCost'] = str(round(CP.steerRateCost,3))
      write_conf = True

    if write_conf:
      self.write_config(self.config)

  def read_config(self):
    self.element_updated = False

    if os.path.isfile('/data/kyd.json'):
      with open('/data/kyd.json', 'r') as f:
        self.config = json.load(f)

      if "steerMax" not in self.config:
        self.config.update({"steerMax":"255"})
        self.config.update({"steerDeltaUp":"3"})
        self.config.update({"steerDeltaDown":"7"})
        self.config.update({"steerDriverAllowance":"50"})
        self.config.update({"steerDriverMultiplier":"2"})
        self.config.update({"steerDriverFactor":"1"})
        self.element_updated = True

      if "cameraOffset" not in self.config:
        self.config.update({"cameraOffset":"0.06"})
        self.element_updated = True
        
      if "steerAngleCorrection" not in self.config:
        self.config.update({"steerAngleCorrection":"0.0"})
        self.element_updated = True

      if "Kp" not in self.config:
        self.config.update({"Kp":"-1"})
        self.config.update({"Ki":"-1"})
        self.config.update({"Kf":"-1"})
        self.element_updated = True
	
      if "steerRatio" not in self.config:
        self.config.update({"steerRatio":"-1"})
        self.config.update({"steerRateCost":"-1"})
        self.config.update({"deadzone":"0.0"})
        self.element_updated = True
	
      if "sR_boost" not in self.config:
        self.config.update({"sR_boost":"0"})
        self.config.update({"sR_BP0":"0"})
        self.config.update({"sR_BP1":"0"})
        self.config.update({"sR_time":"0.1"})
        self.element_updated = True

      if self.element_updated:
        print("updated")
        self.write_config(self.config)

    else:
      self.config = {"steerMax":"255", "steerDeltaUp":"3", "steerDeltaDown":"7", "steerDriverAllowance":"50", "steerDriverMultiplier":"2", "steerDriverFactor":"1", \
      	             "steerAngleCorrection":"0.0", "cameraOffset":"0.06", "Kp":"-1", "Ki":"-1", "Kf":"-1", \
                     "steerRatio":"-1", "steerRateCost":"-1", "deadzone":"0.0", \
                     "sR_boost":"0", "sR_BP0":"0", "sR_BP1":"0", "sR_time":"0.1"}


      self.write_config(self.config)
    return self.config

  def write_config(self, config):
    try:
      with open('/data/kyd.json', 'w') as f:
        json.dump(self.config, f, indent=2, sort_keys=True)
        os.chmod("/data/kyd.json", 0o764)
    except IOError:
      os.mkdir('/data')
      with open('/data/kyd.json', 'w') as f:
        json.dump(self.config, f, indent=2, sort_keys=True)
        os.chmod("/data/kyd.json", 0o764)
