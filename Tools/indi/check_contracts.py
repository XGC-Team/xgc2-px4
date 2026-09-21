#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Static integration regression checks. Not a compiler, scheduler, or flight test."""
from pathlib import Path
import hashlib
import re
import unittest
ROOT = Path(__file__).resolve().parents[2]

def text(path): return (ROOT/path).read_text()
def blob_sha(content):
    b=content.encode(); return hashlib.sha1(b'blob '+str(len(b)).encode()+b'\0'+b).hexdigest()

class Contracts(unittest.TestCase):
    def test_pid_host_restores_exact_pinned_blob(self):
        s=text('src/modules/mc_rate_control/MulticopterRateControl.cpp')
        hook='''
\t\t\t// INDI research diagnostics run AFTER the unchanged PID publications.
\t\t\t// No candidate is written to a control topic in this phase.
\t\t\t_indi_shadow.update(angular_velocity, _rates_setpoint, vehicle_torque_setpoint,
\t\t\t\t\t    _vehicle_control_mode.flag_armed, _maybe_landed || _landed,
\t\t\t\t\t    _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
'''
        self.assertEqual(s.count(hook),1)
        self.assertLess(s.index('_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);'),s.index('_indi_shadow.update('))
        restored=s.replace(hook,'').replace('\t_indi_shadow(this, vtol),\n','').replace('\t_indi_shadow.parametersUpdated();\n','')
        self.assertEqual(blob_sha(restored),'a7a8ed42126dacc63762fa68b29f4806356f6a5e')
        s=text('src/modules/mc_rate_control/MulticopterRateControl.hpp')
        restored=s.replace('\n#include "IndiShadow.hpp"\n','').replace('\tIndiShadow _indi_shadow; ///< read-only research path; no actuator authority\n','')
        self.assertEqual(blob_sha(restored),'76e488305addb66f81561367ca3901ffa300a6ce')

    def test_message_registration_only_adds_one_message(self):
        s=text('msg/CMakeLists.txt')
        self.assertEqual(s.count('\tIndiRateStatus.msg\n'),1)
        self.assertEqual(blob_sha(s.replace('\tIndiRateStatus.msg\n','')),'7eda1cc316a17e96f2a1f1c1c1a6187d12e20114')
        schema=text('msg/IndiRateStatus.msg')
        self.assertIn('uint64 timestamp_sample',schema)
        self.assertIn('bool ready_for_takeover',schema)

    def test_shadow_has_no_actuator_authority(self):
        h=text('src/modules/mc_rate_control/IndiShadow.hpp')
        s=text('src/modules/mc_rate_control/IndiShadow.cpp')
        pubs=re.findall(r'uORB::Publication(?:Multi)?<([^>]+)>',h)
        self.assertEqual(pubs,['indi_rate_status_s'])
        self.assertIn('const vehicle_torque_setpoint_s &issued',s)
        self.assertNotRegex(s,r'issued\.xyz\[[^]]+\]\s*=')
        self.assertIn('status.ready_for_takeover = false;',s)
        self.assertNotIn('ready_for_takeover = true',s)
        self.assertIn('FLAG_MODEL_UNQUALIFIED | indi_rate_status_s::FLAG_FILTER_UNQUALIFIED',s)

    def test_parameter_contract(self):
        s=text('src/modules/mc_rate_control/indi_params.c')
        names=re.findall(r'PARAM_DEFINE_\w+\((\w+),',s)
        self.assertEqual(len(names),12); self.assertEqual(len(set(names)),12)
        self.assertTrue(all(len(n)<=16 for n in names))
        self.assertIn('PARAM_DEFINE_INT32(MC_INDI_MODE, 0)',s)
        for p in ['MC_INDI_GR','MC_INDI_GP','MC_INDI_GY','MC_INDI_TAU']:
            self.assertIn(f'PARAM_DEFINE_FLOAT({p}, 0.0f)',s)
        adapter=text('src/modules/mc_rate_control/IndiShadow.cpp')
        staging=adapter.split('void IndiShadow::applyPending()')[0]
        self.assertNotIn('applyPending();',staging)
        self.assertIn('if (_parameters_pending && !armed) { applyPending(); }',adapter)
        self.assertIn('_mode = (_pending_mode == 1) ? 1 : 0;',adapter)

    def test_math_library_has_no_platform_io(self):
        s=text('src/lib/indi/IndiControl.hpp')+text('src/lib/indi/IndiControl.cpp')
        self.assertNotRegex(s,r'#include.*(?:uORB|drivers|module_params|mavlink|rclcpp)')
        self.assertNotRegex(s,r'\b(?:malloc|calloc|realloc|new|throw)\s*\(')
        self.assertIn('_held_command[i] - _input_model[i]',s)
        self.assertIn('s.timestamp_sample <= _last_sample',s)

if __name__=='__main__': unittest.main(verbosity=2)
